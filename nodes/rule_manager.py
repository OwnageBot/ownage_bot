#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
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
        self.active_rule_db = dict()
        # Database of candidate rules instructed by users
        self.cand_rule_db = dict()
        # Database of action facts (e.g. whether object X can be picked up)
        self.fact_db = dict()
        
        # Subscribers
        self.fact_input_sub = rospy.Subscriber("fact_input", FactMsg,
                                               self.factInputCb)
        self.rule_input_sub = rospy.Subscriber("rule_input", RuleMsg,
                                                self.ruleInputCb)
        # Servers
        # self.lkp_rule_srv = rospy.Service("lookup_rules", LookupRules,
        #                                   self.lookupRulesCb)

    def factInputCb(self, msg):
        """Updates fact database and re-evaluates rule set."""
        # Ignore facts which are not about actions
        if msg.predicate not in self.action_db:
            return
        self.insertFact(msg)
        # Only update ruleset if there are enough facts to induct from
        if len(self.fact_db[msg.predicate]) >= 2:
            # Only update rules that correspond to action
            self.updateRuleSet(msg.predicate)

    def insertFact(self, fact):
        """Insert fact into fact database in appropriate format.
        Fact database is a dict of dicts. The outer dict is indexed by the
        name of the action, the inner dict is indexed by the target.

        Targets are represented by ID for Objects and as a tuple for Points.
        The value stored is the confidence in the fact being true.
        """
        action = self.action_db[fact.predicate]
        if action.name not in self.fact_db:
            self.fact_db[action.name] = dict()
        if len(fact.args) != 1:
            raise TypeError("Action fact should have exactly one argument.")
        tgt = fact.args[0]
        # Make sure target is represented in a hashable way
        if action.tgtype == Object:
            tgt = int(tgt)
        elif action.tgtype == Point:
            tgt = tuple(tgt.split())
        # Overwrite old value if fact already exists
        self.fact_db[action.name][tgt] = fact.confidence

    def ruleInputCb(self. msg):
        """Updates candidate rule database, considers activating them."""
        action = self.action_db[msg.action]
        if action.name not in self.cand_rule_db:
            self.cand_rule_db[action.name] = dict()
        rule = Rule.fromMsg(msg)
        # Overwrites old value if candidate already exists
        self.cand_rule_db[action.name][rule] = msg.confidence
        self.updateRuleSet(action.name)

    def updateRuleSet(self, act_name):
        pass

if __name__ == '__main__':
    rospy.init_node('rule_manager')
    rule_manager = RuleManager()
    rospy.spin()
