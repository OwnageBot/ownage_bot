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

        # Initialize databases with empty dicts/sets
        for a in self.action_db.iterkeys():
            self.active_rule_db[a] = set()
            self.cand_rule_db[a] = dict()
            self.fact_db[a] = dict()

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
        rule = Rule.fromMsg(msg)

        # Overwrites old value if candidate already exists
        self.cand_rule_db[action.name][rule] = msg.confidence
        self.updateRuleSet(action.name, rule)

    def updateRuleSet(self, act_name, cand_rule=None):
        """Evaluates rules for the named action, updates if necessary."""

        if len(self.fact_db[act_name]) > 0:
            # Grow rule set (e.g. add / refine candidate rules)
            self.growRuleSet(act_name, cand_rule)
        elif cand_rule is not None:
            # If there are no facts, just use candidate rules
            self.fact_db[act_name].add(cand_rule)

        # Prune ruleset (e.g. merge rules that can be merged)
        self.pruneRuleSet(act_name, cand_rule)

    def growRuleSet(self, act_name, cand_rule=None):
        """Grow rule set via modified ProbFOIL algorithm."""
        rule_set = self.active_rule_db[act_name]
        fact_set = self.fact_db[act_name]
        
        # Consider adding rules if accuracy is too low
        perf = self.evalRuleSet(rule_set, fact_set)
        while perf.acc < 0.9:
            # Start from candidate rule if available
            new_rule = (cand_rule if cand_rule not is None else
                        self.guessRule(act_name))

            # Repeatedly evaluate and refine rule
            while True:
                # Skip if rule is already in ruleset
                if new_rule in rule_set:
                    new_rule = self.refineRule(new_rule)
                    continue
                # Add rule and evaluate performance
                rule_set.add(new_rule)
                new_perf = self.evalRuleSet(rule_set, fact_set)
                # Check if stopping criterion is true
                if new_perf.fp == 0 or (new_perf.tp - perf.tp == 0):
                    break
                # Refine rule (should select best refinement)
                rule_set.remove(new_rule)
                new_rule = self.refineRule(new_rule)

            # Break if accuracy does not improve, else keep adding more
            new_perf = self.evalRuleSet(rule_set, fact_set)
            if new_perf.acc <= perf.acc:
                # Get rid of rule added in inner loop
                rule_set.remove(new_rule)
                break

    def pruneRuleSet(self, act_name, cand_rule=None):
        """Prunes the active ruleset for the named action."""
        rule_set = self.active_rule_db[act_name]
        fact_set = self.fact_db[act_name]            
            
    def evalRuleSet(self, rule_set, fact_set):
        """Evaluates rule set and returns a performance metric tuple."""
        pass
        
if __name__ == '__main__':
    rospy.init_node('rule_manager')
    rule_manager = RuleManager()
    rospy.spin()
