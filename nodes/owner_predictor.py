#!/usr/bin/env python
import rospy
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics.pairwise import rbf_kernel
from std_srvs.srv import Trigger, TriggerResponse
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class OwnerPredictor:
    """Predicts ownership based on physical and social observation."""
    
    def __init__(self):
        # Set up callback to predict ownership upon new permission input
        self.perm_sub = rospy.Subscriber("perm_input", PredicateMsg,
                                         self.permInputCb)
        # Set up callback to predict ownership upon new object detection
        self.new_obj_sub = rospy.Subscriber("new_object", PredicateMsg,
                                            self.newObjectCb)

        # Publisher for ownership predictions
        self.owner_pub = rospy.Publisher("owner_prediction", ObjectMsg,
                                         queue_size=10)
        
        # Client for looking up active rules
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)
        # Client for looking up tracked objects
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)

        # Weights for perceptual properties in distance metric
        self.w_color = rospy.get_param("~w_color", 30)
        self.w_pos = rospy.get_param("~w_pos", 10)
        self.w_matrix = np.diag([self.w_color] + [self.w_pos] * 3)

        # Logistic regression params and vars for percept-based prediction
        self.regu_strength = 1.0
        self.lr_classifier = LogisticRegression(C=1/self.regu_strength,
                                                fit_intercept=False)
        
    def permInputCb(self, msg):
        """Callback upon receiving permission information about objects."""
        # Ignore perms which are not about actions
        if msg.predicate not in actions.db:
            return
        action = actions.db[msg.predicate]

        # Ignore actions without objects as targets
        if len(msg.bindings) != 1:
            raise TypeError("Action perm should have exactly one argument.")
        if (msg.bindings[0] == objects.Nil.toStr() or action.tgtype == Object):
            return
        obj = Object.fromStr(msg.bindings[0])

        # TODO: Should repeated permissions be ignored??
        
        # Guess ownership and publish prediction
        ownership = self.guessFromPerm(action, obj, msg.truth)
        obj.ownership = ownership
        self.owner_pub.publish(obj.toMsg())

    def newObjectCb(self, msg):
        """Callback upon new object detection."""
        obj = Object.fromMsg(msg)

        # Guess ownership and publish prediction
        ownership = self.guessFromPercepts(obj)
        obj.ownership = ownership
        self.owner_pub.publish(obj.toMsg())
        
    def guessFromPerm(self, act_name, obj, truth):
        """Guess ownership from permission info."""
        rule_set = self.lookupRules(act_name).rule_set
        rule_set = [Rule.fromMsg(r) for r in rule_set]
            
        # Do Bayesian update using potential explanations
        p_owned_prior = obj.ownership
        p_owned_post = dict()
        p_f_owned = dict()
        p_a_owned = dict()
        p_forbidden = Rule.evaluateOr(rule_set, obj)
        p_allowed = 1 - p_forbidden

        for a in Agent.universe():
            obj.ownership = dict(p_owned_prior)
            obj.ownership[a.id] = 1.0
            
            # Find P(forbidden|owned by a) and P(allowed|owned by a)
            p_f_cond = Rule.evaluateOr(rule_set, obj)
            p_a_cond = 1 - p_f_cond
            
            # Find P(forbidden & owned by a) and P(allowed & owned by a)
            p_f_owned[a.id] = p_f_cond *  p_owned_prior[a.id]
            p_a_owned[a.id] = p_a_cond *  p_owned_prior[a.id]
            
        # Compute updated probability of ownership
        for a in Agent.universe():
            # P(owned by a|perm) =
            # P(owned by a|forbidden) P(forbidden|perm) +
            # P(owned by a|allowed) P(allowed|perm)
            p_owned_post[a.id] = \
                (p_f_owned[a.id] / p_forbidden * truth +
                 p_a_owned[a.id] / p_allowed * (1-truth))

        # Reset ownership to original
        obj.ownership = p_owned_prior

        # Return posterior probabilities
        return p_owned_post

    def guessFromPercepts(self, obj):
        """Guess ownership from perception of physical properties."""
        objs = self.listObjects().objects

        # Compute kernel for kernel logistic regression
        kern_mat = self.perceptKern(objs, objs)
        # Tile the matrix to account for uncertainty in labels
        kern_mat = np.tile(kern_mat, (2,2))
        
        # Train a kernel logistic classifier for each possible owner
        ownership = dict()
        for a in Agent.universe():
            weights = np.array([o.ownership[a.id] for o in objs] +
                               [1.0-o.ownership[a.id] for in objs])
            classes = [w >= 0.5 for w in weights]
            self.lr_classifier.fit(kern_mat, classes, sample_weight=weights)
            # Predict ownership of new object
            kern_features = self.perceptKern([obj], objs)
            probs = self.lr_classifier.predict_proba(kern_features)
            ownership[a.id] = probs[0,0]
        
        return ownership

    def perceptDiff(self, o1, o2):
        """Computes raw displacement in perceptual space between objects."""
        col_diff = 1.0 if o1.color != o2.color else 0.0
        p1, p2 = o1.position, o2.position
        pos_diff = [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
        return np.array([col_diff] + pos_diff)

    def perceptKern(self, objs1, objs2, gamma=1.0):
        """Computes RBF kernel matrix for the percept features of objects."""
        diffs = [[self.perceptDiff(o1, o2) for o2 in objs2] for o1 in objs1]
        sq_dists = np.array([[np.dot(d, d) for d in row] for row in diffs])
        return np.exp(-gamma * sq_dists)
        
