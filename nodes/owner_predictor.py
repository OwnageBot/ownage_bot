#!/usr/bin/env python
import rospy
from collections import namedtuple
from std_srvs.srv import Trigger, TriggerResponse
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class OwnerPredictor:
    """Predicts ownership based on social observation."""
    
    def __init__(self):
        self.perm_sub = rospy.Subscriber("perm_input", PredicateMsg,
                                         self.permInputCb)
        self.owner_pub = rospy.Publisher("owner_prediction", ObjectMsg,
                                          queue_size=10)
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)


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

        # Guess ownership and publish prediction
        ownership = self.guessFromPerm(action, obj, msg.truth)
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

        
