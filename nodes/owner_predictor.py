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

    def guessFromPerm(act_name, obj, truth):
        """Guess ownership from permission info."""
        rule_set = self.lookupRules(act_name).rule_set
        rule_set = [Rule.fromMsg(r) for r in rule_set]

        # Handle 'allow' permissions
        if truth < 0.5:
            # TODO
            return

        # Handle 'forbid' permissions

        explanations = []
        # Find all potential explanations
        for r in rule_set:
            # Check if rule has any ownedBy predicates
            # If no ownedBy predicates, check if rule is true
            # Return if true -- can't do inference
            # Discard all ownedBy predicates in rule
            # Continue if rule evaluates to false
            # Add to list of potential explanations if true
            explanations.append(r)
            
        # Do Bayesian update using potential explanations
        # Compute total probability that action was forbidden
        for a in Agent.universe():
            pass

        # Compute updated probability of ownership
        for a in Agent.universe():
            pass

        # TODO: switch to DS representation of ownership??

        
