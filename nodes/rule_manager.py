#!/usr/bin/env python
import rospy
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleManager:
    """Manages, updates and learns (ownership) rules."""
    def __init__(self):
        # Databases of available predicates and actions
        self.predicate_db = dict(zip([p.name for p in predicates.db],
                                     predicates.db))
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))

        # Database of actively-followed rules
        self.rule_db = dict()
        # Database of candidate rules instructed by users
        self.instruction_db = dict()
        # Database of action facts (e.g. whether object X can be picked up)
        self.fact_db = dict()
        
        # Subscribers
        self.fact_sub = rospy.Subscriber("fact", FactMsg, self.factCb)
        self.instruction_sub = rospy.Subscriber("instruction", RuleMsg,
                                                self.instructionCb)
        # Servers
        # self.lkp_rule_srv = rospy.Service("lookup_rules", LookupRules,
        #                                   self.lookupRulesCb)

    def factCb(self, msg):
        pass

    def instructionCb(self. msg):
        pass
