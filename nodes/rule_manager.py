#!/usr/bin/env python
import rospy
from collections import namedtuple
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

# Named tuple for performance metric info
PerfMetric = namedtuple('PerfMetric',
                        ['tp', 'tn', 'fp', 'fn',
                         'prec', 'rec', 'acc', 'm_est'])

class RuleManager:
    """Manages, updates and learns (ownership) rules."""
    
    def __init__(self):
        # Learning parameters
        self.grow_thresh = rospy.get_param("grow_threshold", 0.9)
        self.add_fact_thresh = 0.9
        self.sub_fact_thresh = 0.9
        self.add_rule_thresh = 0.1
        self.sub_rule_thresh = 0.1
        self.max_cand = 3
        
        # Databases of available predicates and actions
        self.predicate_db = dict(zip([p.name for p in predicates.db],
                                     predicates.db))
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))

        # Database of actively-followed rules
        self.active_rule_db = dict()
        # Database of rules given by users
        self.given_rule_db = dict()
        # Database of action facts (e.g. whether object X can be picked up)
        self.fact_db = dict()

        # Initialize databases with empty dicts/sets
        for a in self.action_db.iterkeys():
            self.active_rule_db[a] = set()
            self.given_rule_db[a] = dict()
            self.fact_db[a] = dict()

        # Subscribers
        self.fact_input_sub = rospy.Subscriber("fact_input", PredicateMsg,
                                               self.factInputCb)
        self.rule_input_sub = rospy.Subscriber("rule_input", RuleMsg,
                                                self.ruleInputCb)
        # Servers
        # self.lkp_rule_srv = rospy.Service("lookup_rules", LookupRules,
        #                                   self.lookupRulesCb)

    def factInputCb(self, msg):
        """Updates fact database, then tries to cover new fact."""
        # Ignore facts which are not about actions
        if msg.predicate not in self.action_db:
            return

        action = self.action_db[fact.predicate]
        if len(msg.args) != 1:
            raise TypeError("Action fact should have exactly one argument.")
        tgt = msg.args[0]

        # Make sure target is represented in a hashable way
        if action.tgtype == Object:
            # Object properties are looked up and stored at time of receipt
            tgt = Object.fromStr(tgt)
        elif action.tgtype == Point:
            tgt = tuple(tgt.split()) #TODO: Make location targets hashable

        # Overwrite old value if fact already exists
        self.fact_db[action.name][tgt] = msg.truth
        # Update rules to accomodate new fact
        self.accomFact(action.name, tgt, msg.truth)

    def ruleInputCb(self, msg):
        """Updates given rule database, considers activating them."""
        action = self.action_db[msg.action]
        rule = Rule.fromMsg(msg)

        # Overwrites old value if given rule already exists
        self.given_rule_db[action.name][rule] = msg.truth
        # Accomdate the given rule
        self.accomRule(rule, truth)
        
    def accomFact(self, act_name, tgt, truth):
        """Tries to accommodate the new fact by modifying rule base."""
        rule_set = self.active_rule_db[act_name]
        prediction = Rule.evaluateOr(rule_set, tgt)       

        if truth >= 0.5 and prediction < 0.5:
            self.coverFact(act_name, tgt, truth)
        elif truth < 0.5 and prediction >= 0.5:
            self.uncoverFact(act_name, tgt, truth)

    def coverFact(self, act_name, tgt, truth):
        """Covers positive fact via general-to-specific search for a rule."""
        rule_set = self.active_rule_db[act_name]
        fact_set = self.fact_db[act_name]
        neg_facts = {k: v for k, v in fact_set.items() if v < 0.5}

        # Search only for inactive rules (pointless to refine active rules) 
        inactive_f = lambda r : r not in rule_set
        # Candidate rules must cover the new fact
        cover_f = lambda r : r.evaluate(tgt) >= truth
        # Compute score as false positive value for each candidate rule
        score_f = lambda r : sum([max(r.evaluate(n_tgt) - n_val, 0) for
                                  n_tgt, n_val in neg_facts.items()])

        # Search for rule starting with empty rule
        init_rule = Rule(self.action_db[act_name], conditions=[])
        score_thresh = self.add_fact_thresh
        new_rule, new_score = self.ruleSearch(init_rule, score_thresh,
                                              score_f, [inactive_f, cover_f])

        # Add new rule if one is found
        if new_rule != init_rule:
            rule_set.add(new_rule)
        else:
            print "Cannot find rule that to cover new fact."
            
    def uncoverFact(self, act_name, tgt, truth):
        """Uncover negative fact by refining overly general rules."""
        rule_set = self.active_rule_db[act_name]
        fact_set = self.fact_db[act_name]

        # Find set of high-certainty covering rules
        cover_rules = [r for r in rule_set if r.evaluate(tgt) >= 0.5]

        # Candidate rules must *not* cover the new fact
        uncover_f = lambda r : r.evaluate(tgt) < truth

        # Refine each rule to uncover new fact while still covering others
        for init_rule in cover_rules:
            # Find set of covered positive facts
            pos_facts = {k: v for k, v in fact_set.items()
                         if v >= 0.5 and init_rule.evaluate(k) >= 0.5}

            # Compute score as false negative value for each candidate rule
            score_f = lambda r : sum([max(p_val - r.evaluate(p_tgt), 0) for
                                      p_tgt, p_val in pos_facts.items()])

            # Search for rule starting with covering rule
            score_thresh = self.sub_fact_thresh
            new_rule, new_score = self.ruleSearch(init_rule, score_thresh,
                                                  score_f, [uncover_f])

            # Replace old rule with new rule if one is found
            if new_rule != init_rule:
                rule_set.remove(init_rule)
                rule_set.add(new_rule)
            else:
                print "Cannot refine rule to uncover new fact."

    def accomRule(self, given_rule, truth):
        """Tries to accommodate the given rule by modifying rule base."""
        if truth >= 0.5:
            self.coverRule(given_rule, truth)
        else:
            self.uncoverRule(given_rule, truth)

    def coverRule(self, given_rule, truth):
        """Cover given positive rule if not already covered."""
        rule_set = self.active_rule_db[given_rule.action.name]
        fact_set = self.fact_db[given_rule.action.name]
        neg_facts = {k: v for k, v in fact_set.items() if v < 0.5}
        n_neg = len(neg_facts)
        
        # Do nothing if given rule is specialization of an active rule
        for r in rule_set:
            if r.conditions.issubset(given_rule.conditions):
                return

        # Score candidate rules according to false positive value 
        score_f = lambda r : sum([max(r.evaluate(tgt) - val, 0) for
                                  tgt, val in neg_facts.items()])
        given_score = score_f(given_rule)

        # Terminate early if there are no false positives
        if given_score == 0:
            rule_set.add(given_rule)
            return
            
        # Specialize rule so that false positives are minimized
        new_rule, new_score  = self.ruleSearch(given_rule, given_score,
                                               score_f)

        # Add specialized rule if false positive fraction is low enough
        if new_score <= self.add_rule_thresh * n_neg:
            rule_set.add(new_rule)
        else:
            print "Cannot add given rule without too many false positives."
        
    def uncoverRule(self, given_rule, truth):
        """Uncover negative rule by refining existing rules."""
        rule_set = self.active_rule_db[given_rule.action.name]
        fact_set = self.fact_db[given_rule.action.name]

        # Find set of active rules with non-empty intersections
        cover_rules = [r for r in rule_set if
                       all([c.negate() not in r.conditions
                            for c in given_rule.conditions])]

        # Refine covering rules to uncover intersection with given rule
        for init_rule in cover_rules:
            # Get logical intersection of given rule with covering rule
            inter_rule = Rule.intersect(given_rule, init_rule,
                                        negate_check=False)

            # If intersection is equal to covering rule, remove covering rule
            if init_rule == inter_rule:
                rule_set.remove(init_rule)
                continue
            
            # Find positive facts covered by init_rule but not inter_rule
            pos_facts = {k: v for k, v in fact_set.items()
                         if v >= 0.5 and init_rule.evaluate(k) >= 0.5
                         and inter_rule.evaluate(k) < 0.5}
            n_pos = len(pos_facts)
            
            # Candidate rules must not cover the intersection
            uncover_f = (lambda r :
                         not r.conditions.issubset(inter_rule.conditions))
            # Score candidate rules according to false negative value
            score_f = lambda r, fs : sum([max(val - r.evaluate(tgt), 0) for
                                         tgt, val in fs.items()])

            # Find refinements that cover previously covered positive facts
            new_set = set()
            remain_facts = pos_facts
            while len(remain_facts) > 0:
                # Find refinement that best covers remaining facts
                new_rule, new_score = \
                    self.ruleSearch(init_rule, len(remain_facts),
                                    lambda r : score_f(r, remain_facts),
                                    [uncover_f])
                if new_rule == init_rule:
                    print "Cannot find any more suitable refinements."
                    break
                new_set.add(new_rule)
                # Remove sufficiently covered facts
                remain_facts = {k : v for k, v in remain_facts.items()
                                if new_rule.evaluate(k) > 0.5}

            # Replace old rule with set of refinements
            if len(new_set) != 0:
                rule_set.remove(init_rule)
                rule_set.update(new_set)
            else:
                print "Cannot find set of suitable refinements."            
            
    def ruleSearch(self, init_rule, score_thresh, score_f, filters=[]):
        """Performs general to specific search for minimal-scoring rule."""
        best_rule, best_score = init_rule, score_thresh
        cand_rules = [best_rule]
            
        while len(cand_rules) > 0:
            # Construct list of refinements from previous candidates
            new_rules = sum([self.refineRule(r) for r in cand_rules])
            # Select rules which match filters
            for f in filters:
                new_rules = filter(new_rules, f)
            # Return if no more rules
            if len(new_rules) == 0:
                return best_rule
            # Compute and sort by scores
            scores = [score_f(r) for r in new_rules]
            sort_rules = sorted(zip(new_rules, scores), key=lambda p:p[1])
            # Select top few candidates for next round of refinement
            cand_rules = [r for r, s in sort_rules[0:self.max_cand]]
            # Check if best candidate beats best rule
            if sort_rules[0][1] < best_score:
                best_rule = cand_rules[0]
        return best_rule, best_score
                             
    def pruneRuleSet(self, act_name, given_rule=None):
        """Prunes the active ruleset for the named action."""
        rule_set = self.active_rule_db[act_name]
        fact_set = self.fact_db[act_name]            
            
    def evalRuleSet(self, rule_set, fact_set):
        """Evaluates rule set and returns a performance metric tuple."""
        n_facts = len(fact_set)
        n_true = sum(fact_set.values())
        n_false = n_facts - n_true
        tp, tn, fp, fn = 0.0, 0.0, 0.0, 0.0
        for tgt, truth in fact_set.items():
            prediction = Rule.evaluateOr(rule_set, tgt)
            tpi, tni = min(truth, predict), min(1-truth, 1-predict)
            fpi, fni = max(0, (1-truth)-tni), max(0, truth-tpi)
            tp, tn = tp + tpi, tn + tni
            fp, fn = fp + fpi, fn + fni
        prec = tp / (tp + fp)
        rec = tp / (tp + fn)
        acc = (tp + tn) / n
        m_est = (tp + self.m_param * n_true/n_false) / (tp + fp)
        return PerfMetric(tp, tn, fp, fn, prec, rec, acc, m_est)
            
if __name__ == '__main__':
    rospy.init_node('rule_manager')
    rule_manager = RuleManager()
    rospy.spin()
