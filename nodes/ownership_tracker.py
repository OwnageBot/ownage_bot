#!/usr/bin/env python
import rospy
import threading
import numpy as np
from collections import defaultdict
from sklearn.kernel_approximation import Nystroem
from sklearn.linear_model import LogisticRegression
from std_srvs.srv import *
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from object_tracker import ObjectTracker

class OwnershipTracker(ObjectTracker):
    """Tracks ownership based on physical and social observation."""
    
    def __init__(self):
        super(OwnershipTracker, self).__init__()

        # Flag to disable rule-based inference
        self.disable_inference =\
            rospy.get_param("~disable_inference", False)
        # Flag to disable percept-based extrapolation
        self.disable_extrapolate =\
            rospy.get_param("~disable_extrapolate", False)

        # Database of ownership claims
        self.claim_db = defaultdict(dict)
        # Database of ownership predictions
        self.predict_db = defaultdict(dict)
        # Database of action permissions
        self.perm_db = defaultdict(dict)

        # Do not use inferred ownership as input to ownership inference
        Object.use_inferred = False
        
        # Lock to ensure callbacks update ownership synchronously
        self.owner_lock = threading.Lock()
        
        # Set up callback to predict ownership upon new permission input
        self.perm_sub = rospy.Subscriber("perm_input", PredicateMsg,
                                         self.permInputCb)
        # Set up callback to handle ownership claims
        self.owner_sub = rospy.Subscriber("owner_input", PredicateMsg,
                                          self.ownerClaimCb)
        # Set up callback to predict ownership upon new object detection
        self.new_agt_sub = rospy.Subscriber("new_agent", AgentMsg,
                                            self.newAgentCb)

        # Services to stop owner prediction
        self.dis_infer_srv = rospy.Service("disable_inference", SetBool,
                                           self.disableInferenceCb)
        self.dis_extra_srv = rospy.Service("disable_extrapolate", SetBool,
                                           self.disableExtrapolateCb)

        # Reset service for objects and ownership information
        self.rst_own_srv = rospy.Service("reset_ownership", Trigger,
                                         self.resetOwnershipCb)
        
        # Clients for looking up active rules and permissions
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)
        self.lookupPerm = rospy.ServiceProxy("lookup_perm", LookupPerm)
        
        # How much to trust ownership claims
        self.claim_trust = rospy.get_param("~claim_trust", 1.0)
        # Default ownership prior
        self.default_prior = rospy.get_param("~default_prior", 0.5)

        # Feature weights for percept-based prediction
        self.col_weight = rospy.get_param("~col_weight", 0.5)
        self.pos_weight = rospy.get_param("~pos_weight", 1.0)
        self.time_weight = rospy.get_param("~time_weight", 0.05)
        
        # Logistic regression params and objects for percept-based prediction
        self.reg_strength = rospy.get_param("~reg_strength", 0.1)
        self.max_features = rospy.get_param("~max_features", 20)
        self.nys = dict()
        self.log_reg = dict()
        
    def disableInferenceCb(self, req):
        """Disables rule-based ownership inference."""
        self.disable_inference = req.data
        return SetBoolResponse(True, "")

    def disableExtrapolateCb(self, req):
        """Disables percept-based ownership extrapolation."""
        self.disable_extrapolate = req.data
        return SetBoolResponse(True, "")

    def resetObjectsCb(self, req):
        """Clears the object databases."""
        super(OwnershipTracker, self).resetObjectsCb(req)
        self.resetOwnershipCb(req)
        return TriggerResponse(True, "")
    
    def resetOwnershipCb(self, req):
        """Resets database of ownership claims and predictions."""
        self.owner_lock.acquire()
        # Clear all claims, predictions, inferences
        self.claim_db.clear()
        self.predict_db.clear()
        self.perm_db.clear()
        for obj in self.object_db.itervalues():
            obj.ownership.clear()
            obj.inferred.clear()
        self.owner_lock.release()
        return TriggerResponse(True, "")

    def permInputCb(self, msg):
        """Callback upon receiving permission information about objects."""
        # Do nothing if inference is disabled
        if self.disable_inference:
            return
        
        # Ignore perms which are not about actions
        if msg.predicate not in actions.db:
            return
        act = actions.db[msg.predicate]

        # Ignore actions without objects as targets
        if len(msg.bindings) != 1:
            raise TypeError("Action perm should have exactly one argument.")
        if (msg.bindings[0] == objects.Nil.toStr() or act.tgtype != Object):
            raise TypeError("Action perm should have object as argument.")
        try:
            obj = Object.fromStr(msg.bindings[0])
        except ValueError:
            raise ValueError("Could not resolve object ID - needs to be int.")

        # Store permission
        self.perm_db[act.name][obj.id] = msg.truth
        
        # Infer ownership
        self.owner_lock.acquire()
        self.inferOwnership(obj_ids=[obj.id])
        self.owner_lock.release()
    
    def ownerClaimCb(self, msg):
        """Callback upon receiving claim of ownership about object."""
        # Unpack object and owner identity from isOwned predicate
        pred = Predicate.fromMsg(msg)
        obj = pred.bindings[0]
        agent = pred.bindings[1]
        if pred.name != predicates.OwnedBy.name:
            return
        if not isinstance(obj, Object):
            return
        if not isinstance(agent, Agent):
            # TODO: Handle non-specific and group ownership claims
            return

        # Ignore claim if object not in database (i.e. not tracked)
        if obj.id not in self.object_db:
            return
        
        # Do nothing if agent is not recognized
        if agent.id not in self.claim_db or agent.id not in self.predict_db:
            rospy.logwarn("Ownership claim made for unknown agent {}...".\
                          format(agent.id))
            return

        self.owner_lock.acquire()
        
        # Compute ownership probability as product of trust and truth value
        p_owned = self.claim_trust * msg.truth
        if pred.negated:
            p_owned = 1 - p_owned
        self.claim_db[agent.id][obj.id] = p_owned
        self.object_db[obj.id].ownership[agent.id] = p_owned

        # Retrain predictor and update prediction probabilities
        if not self.disable_extrapolate:
            self.trainPredictor(agent_ids=[agent.id])
            self.predictOwnership(agent_ids=[agent.id])
        # Use new prior probabilities to perform inference    
        if self.disable_inference:
            for obj in self.object_db.itervalues():
                obj.inferred = dict(obj.ownership)
        else:
            self.inferOwnership()
        self.owner_lock.release()
                
    def newAgentCb(self, msg):
        """Callback upon new agent introduction."""
        rospy.loginfo("Agent {} introduced, guessing ownership...".\
                      format(msg.id))
        self.owner_lock.acquire()
        # Add dictionary of ownership claims and predictions for new agent
        self.claim_db[msg.id] = dict()
        self.predict_db[msg.id] = dict()
        # Construct ownership classifier for new agent
        self.nys[msg.id] = Nystroem(kernel='precomputed', random_state=0)
        self.log_reg[msg.id] = LogisticRegression(C=1/self.reg_strength,
                                                  solver='newton-cg')
        # Default ownership probability to priors
        self.guessOwnership(agent_ids=[msg.id])
        # Use new prior probabilities to perform inference    
        if not self.disable_inference:
            self.inferOwnership()
        else:
            for obj in self.object_db.itervalues():
                obj.inferred = dict(obj.ownership)
        self.owner_lock.release()

    def newObjectCb(self, o_id):
        """Callback upon insertion of new object."""
        self.owner_lock.acquire()
        # Predict ownership of new object
        if self.disable_extrapolate:
            self.guessOwnership(obj_ids=[o_id])
        else:
            self.predictOwnership(obj_ids=[o_id])
        # Make sure inferred probabilities are in sync
        obj = self.object_db[o_id]
        obj.inferred = dict(obj.ownership)
        self.owner_lock.release()
        
    def guessOwnership(self, obj_ids=None, agent_ids=None):
        """Guess probability of ownership using default prior."""
        if obj_ids is None:
            obj_ids = self.object_db.keys()
        if agent_ids is None:
            agent_ids = self.predict_db.keys()
        for o_id in obj_ids:
            if self.object_db[o_id].is_avatar:
                continue
            for a_id in agent_ids:
                self.predict_db[a_id][o_id] = self.default_prior
                self.object_db[o_id].ownership[a_id] = self.default_prior
            
    def inferOwnership(self, obj_ids=None):
        """Infer ownership from permissions and rules."""
        if obj_ids is None:
            obj_ids = self.object_db.keys()
        obj_ids = [i for i in obj_ids if not self.object_db[i].is_avatar]
        
        # Lookup rules in advance
        rule_db = dict()
        for act in actions.db.itervalues():
            if act.tgtype != Object:
                continue
            rule_set = self.lookupRules(act.name).rule_set
            rule_set = [Rule.fromMsg(r) for r in rule_set]
            if len(rule_set) == 0:
                continue
            rule_db[act.name] = rule_set
        
        for o_id in obj_ids:
            # Copy object properties
            obj = self.object_db[o_id].copy()
            
            # Build initial prior from claims and predictions
            p_owned_init = dict()
            for a_id, predict_db in self.predict_db.iteritems():
                p_owned_init[a_id] = predict_db.get(o_id, self.default_prior)
            for a_id, claim_db in self.claim_db.iteritems():
                if o_id in claim_db:
                    p_owned_init[a_id] = claim_db[o_id]
            p_owned_prior = dict(p_owned_init)
            p_owned_post = dict()
            
            # Do Bayesian inference for each rule set
            for act_name, rule_set in rule_db.iteritems():
                # Do not infer if no permission has been given
                if o_id not in self.perm_db[act_name]:
                    continue
                # Lookup relevant permission
                p_forbid_perm = self.perm_db[act_name][o_id]

                # Compute prior probability of action being forbidden
                obj.ownership = dict(p_owned_prior)
                p_forbid = Rule.evaluateOr(rule_set, obj)
                p_allow = 1 - p_forbid

                # Intialize posterior and conditional probabilties
                p_f_owned = dict()
                p_a_owned = dict()
                
                # Compute conditional and joint probabilities
                for a_id in p_owned_prior.iterkeys():
                    # Suppose that obj is owned by agent a_id
                    obj.ownership = dict(p_owned_prior)
                    obj.ownership[a_id] = 1.0
            
                    # Find P(forbid|ownedBy a) and P(allow|ownedBy a)
                    p_f_cond = Rule.evaluateOr(rule_set, obj)
                    p_a_cond = 1 - p_f_cond
            
                    # Find P(forbid & ownedBy a) and P(allow & ownedBy a)
                    p_f_owned[a_id] = p_f_cond *  p_owned_prior[a_id]
                    p_a_owned[a_id] = p_a_cond *  p_owned_prior[a_id]
            
                # Compute posterior probability of ownership
                for a_id in p_owned_prior.iterkeys():
                    # P(ownedBy a|perm) =
                    # P(ownedBy a|forbid) P(forbid|perm) +
                    # P(ownedBy a|allow)  P(allow|perm)
                    p_owned_post[a_id] = 0
                    if p_forbid > 0:
                        p_owned_post[a_id] += (p_f_owned[a_id] / p_forbid *
                                               p_forbid_perm)
                    if p_allow > 0:
                        p_owned_post[a_id] += (p_a_owned[a_id] / p_allow *
                                               (1-p_forbid_perm))

                # Use posterior as prior for next rule set
                p_owned_prior = dict(p_owned_post)
                
            # Set inferred ownership probabilities to final posterior
            if o_id in self.object_db:
                self.object_db[o_id].inferred = dict(p_owned_prior)
            
    def predictOwnership(self, obj_ids=None, agent_ids=None):
        """Predict ownership of objects from physical percepts."""
        # Predict ownership for all agents if none are specified 
        if agent_ids is None:
            agent_ids = self.predict_db.keys()
        if len(agent_ids) == 0:
            return
        
        # Train the classifier for each possible owner
        for a_id in agent_ids:
            # Default to uninformed prior if too few training points
            if len(self.claim_db[a_id]) <= 1:
                self.guessOwnership(obj_ids, [a_id])
                continue

            # Use claimed objects as training set
            train = [o.copy() for o in self.object_db.values()
                     if o.id in self.claim_db[a_id] and not o.is_avatar]
            # Predict ownership of unclaimed objects
            test = [o.copy() for o in self.object_db.values()
                    if o.id not in self.claim_db[a_id] and not o.is_avatar]
            # Only predict ownership for specified objects (if unclaimed)
            if obj_ids is not None:
                test = [o for o in test if o.id in obj_ids]
            # Skip if there's nothing to predict for
            if len(test) == 0:
                continue
                                      
            # Predict new probabilities
            K_test = self.perceptKern(test, train, agent_id=a_id)
            X_test = self.nys[a_id].transform(K_test)
            new_probs = self.log_reg[a_id].predict_proba(X_test)
            new_probs = list(new_probs[:,1])

            # Update probabilities of ownership
            for i, o in enumerate(test):
                self.predict_db[a_id][o.id] = new_probs[i]
                self.object_db[o.id].ownership[a_id] = new_probs[i]

    def trainPredictor(self, agent_ids=None):
        # Train predictor for all agents with claims if none are given
        if agent_ids is None:
            agent_ids = self.claim_db.keys()
        if len(agent_ids) == 0:
            return

        # Train the predictor for each possible owner
        for a_id in agent_ids:
            # Check to make sure there are enough points to train on
            if len(self.claim_db[a_id]) <= 1:
                continue

            # Use claimed objects as training set
            train = [o.copy() for o in self.object_db.values()
                     if o.id in self.claim_db[a_id] and not o.is_avatar]
            
            # Set kernel approximation dims to number of training samples
            self.nys[a_id].n_components = len(train)

            # Compute Gram matrix and kernel map
            K = self.perceptKern(train, train, agent_id=a_id)
            X = self.nys[a_id].fit_transform(K)
            
            # Duplicate samples to account for uncertainty in class labels
            X = np.tile(X, [2,1])
            y = [True] * len(train) + [False] * len(train)

            # Weight samples according to the certainty of ownership claims
            weights = [self.claim_db[a_id][o.id] for o in train]
            weights = np.array(weights + [1.0-w for w in weights])

            # Train the logistic regression classifier
            self.log_reg[a_id].fit(X, y, sample_weight=weights)

                
    def perceptDiff(self, o1, o2, agent_id=None):
        """Computes raw displacement in perceptual space between objects."""
        col_diff = 1.0 if o1.color != o2.color else 0.0
        p1, p2 = o1.position, o2.position
        pos_diff = np.array([p1.x-p2.x, p1.y-p2.y, p1.z-p2.z])
        time_diff = (0.0 if agent_id is None else
                     (o2.t_last_actions.get(agent_id, self.t_init) -
                      o1.t_last_actions.get(agent_id, self.t_init)).to_sec())
        col_diff *= self.col_weight
        pos_diff *= self.pos_weight
        time_diff *= self.time_weight
        return np.concatenate([[col_diff], pos_diff, [time_diff]])

    def perceptKern(self, objs1, objs2, agent_id=None, gamma=1.0):
        """Computes RBF kernel matrix for the percept features of objects."""
        diffs = [[self.perceptDiff(o1, o2, agent_id)
                  for o2 in objs2] for o1 in objs1]
        sq_dists = np.array([[np.dot(d, d) for d in row] for row in diffs])
        return np.exp(-gamma * sq_dists) 

    def certaintyCheck(self, p_old, p_new):
        """Checks if new value will reduce certainty by too much."""
        return ((abs(p_old-self.default_prior) -
                 abs(p_new-self.default_prior)) <= 0)
    
if __name__ == '__main__':
    rospy.init_node('ownership_tracker')
    OwnershipTracker()
    rospy.spin()
