#!/usr/bin/env python
import rospy
import random
from std_srvs.srv import *
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleInstructor(object):
    """Automatically teaches rules by example or direct instruction."""

    def __init__(self):
        # Learning mode (by rule, permission, or script)
        self.mode = rospy.get_param("~mode", "by_perm")

        # Various rates
        self.pub_rate = rospy.Rate(rospy.get_param("~pub_rate", 10))
        self.iter_wait = rospy.get_param("~iter_wait", 0.5)
            
        # Publishers
        self.owner_pub = rospy.Publisher("owner_input", PredicateMsg,
                                         queue_size=10)
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)
        
        # Servers
        self.getObjects = rospy.ServiceProxy("simulation/all_objects",
                                             ListObjects)
        self.getAgents = rospy.ServiceProxy("simulation/all_agents",
                                            ListAgents)
        self.setAgents = rospy.ServiceProxy("set_agents", SendAgents)
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

        # Services that reset various databases
        self.reset = dict()
        self.reset["perms"] = rospy.ServiceProxy("reset_perms", Trigger)
        self.reset["rules"] = rospy.ServiceProxy("reset_rules", Trigger)
        self.reset["objects"] = rospy.ServiceProxy("reset_objects", Trigger)
        self.reset["agents"] = rospy.ServiceProxy("reset_agents", Trigger)
        self.reset["simulation"] = rospy.ServiceProxy("simulation/reset",
                                                      Trigger)
        
        # Services that freeze/unfreeze various databases
        self.freeze = dict()
        self.freeze["perms"] = rospy.ServiceProxy("freeze_perms", SetBool)
        self.freeze["rules"] = rospy.ServiceProxy("freeze_rules", SetBool)

        # Services that disable/enable various functions
        self.disable = dict()
        self.disable["inference"] = \
            rospy.ServiceProxy("disable_inference", SetBool)
        self.disable["extrapolate"] = \
            rospy.ServiceProxy("disable_extrapolate", SetBool)
        
    def loadInstructions(self):
        """Load instructions (should be called after introducing agents)."""
        if self.mode == "by_script":
            self.loadScript()
        else:
            self.loadRules()
        
    def loadRules(self):
        """Loads rules from parameter server."""
        rule_strs = rospy.get_param("~rules", [])
        rule_msgs = [parse.asRule(s) for s in rule_strs]
        if None in rule_msgs:
            raise SyntaxError("Rules were in the wrong syntax")
        rules = [Rule.fromMsg(m) for m in rule_msgs]
        rule_db = dict()
        for r in rules:
            if r.action.name not in rule_db:
                rule_db[r.action.name] = set()
            rule_db[r.action.name].add(r)
        self.rule_db = rule_db

    def loadScript(self):
        """Loads script from parameter server."""
        script_strs = rospy.get_param("~script", [])
        script_msgs = []
        for s in script_strs:
            msg = parse.asPerm(s)
            if msg:
                script_msgs.append(msg)
                continue
            msg = parse.asRule(s)
            if msg:
                script_msgs.append(msg)
                continue
            if msg is None:
                raise SyntaxError("Script was in the wrong syntax")
        self.script_msgs = script_msgs
        
    def initOnline(self):
        """Sets up subscribers for online instruction."""
        pass

    def introduceAgents(self):
        """Looks up all simulated agents and introduces them to the tracker."""
        agents = self.getAgents().agents
        self.setAgents(agents)

    def introduceOwners(self, fraction=1.0, own_mean=1.0,
                        own_dev=0.0, objs=None):
        """Provides ownership labels for a random subset of the objects.

        fraction - Fraction of objs for which owner labels will be given.
        own_mean - Average probability of ownership for given labels
        own_dev - Half-range of ownership probability for the labels
        objs - List of objects from which the fraction will be sampled.
        """
        # Load objects from simulator if not provided
        if objs is None:
            objs = [Object.fromMsg(m) for m in self.getObjects().objects]
            objs = [o for o in objs if not o.is_avatar]
        # Sample random fraction objects
        objs = random.sample(objs, int(fraction * len(objs)))
        for o in objs:
            for agent_id, p_owned in o.ownership.iteritems():
                # Make ownership labels uncertain
                p_owned = own_mean if p_owned > 0.5 else (1-own_mean)
                p_owned += random.uniform(-own_dev, +own_dev)
                p_owned = min(max(p_owned, 0.0), 1.0)
                msg = PredicateMsg(predicate=predicates.OwnedBy.name,
                                   bindings=[o.toStr(), str(agent_id)],
                                   negated=False, truth=p_owned)
                self.pub_rate.sleep()
                self.owner_pub.publish(msg)
        return objs
            
    def batchInstruct(self):
        """Teaches all information in one batch."""
        if self.mode == "by_perm":
            self.batchPermInstruct()
        elif self.mode == "by_rule":
            self.batchRuleInstruct()
        elif self.mode == "by_script":
            self.batchScriptInstruct()
        else:
            rospy.logerror("Instructor mode %s unrecognized", mode)
            
    def batchPermInstruct(self, fraction=1.0, objs=None):
        """Provides all object-specific permissions in one batch.
        
        fraction - Fraction of objs for which permissions will be given.
        objs - List of objects from which the fraction will be sampled.
        """
        # Load objects from simulator if not provided
        if objs is None:
            objs = [Object.fromMsg(m) for m in self.getObjects().objects]
            objs = [o for o in objs if not o.is_avatar]
        # Sample random fraction of (given objects)
        objs = random.sample(objs, int(fraction * len(objs)))
        # Feed permissions for each action separately
        for act_name, rule_set in self.rule_db.iteritems():
            for o in objs:
                truth = Rule.evaluateOr(rule_set, o)
                perm = PredicateMsg(predicate=act_name,
                                    bindings=[o.toStr()],
                                    truth=truth)
                self.pub_rate.sleep()
                self.perm_pub.publish(perm)

    def batchRuleInstruct(self):
        """Publish all known rules in one batch."""
        for rule_set in self.rule_db.values():
            for r in rule_set:
                self.pub_rate.sleep()
                self.rule_pub.publish(r.toMsg())

    def batchScriptInstruct(self):
        """Publish instruction messages in exact order of the script."""
        for msg in self.script_msgs:
            self.pub_rate.sleep()
            if isinstance(msg, PredicateMsg):
                self.perm_pub.publish(msg)
            elif isinstance(msg, RuleMsg):
                self.rule_pub.publish(msg)

    def evaluateRules(self):
        """Evalutes accuracy of learned rules against actual rules."""
        objs = [Object.fromMsg(m) for m in self.getObjects().objects]
        objs = [o for o in objs if not o.is_avatar]
        rule_acc = dict()
        print "Rule\t\tAccuracy"
        for act_name, actual_rules in self.rule_db.iteritems():
            learned_rules = [Rule.fromMsg(m) for m in
                             self.lookupRules(act_name).rule_set]
            rule_acc[act_name] = 0.0
            for o in objs:
                actual_perms = Rule.evaluateOr(actual_rules, o)
                learned_perms = Rule.evaluateOr(learned_rules, o)
                correct = (actual_perms-0.5)*(learned_perms-0.5) > 0
                rule_acc[act_name] += correct
            rule_acc[act_name] /= len(objs)
            print "{}\t\t{}".format(act_name, rule_acc[act_name])
        avg_rule_acc = sum(rule_acc.values()) / len(rule_acc)
        print "Average rule accuracy: {}".format(avg_rule_acc)
        return avg_rule_acc, rule_acc

    def resetAll(self):
        """Resets all database to prepare for next instruction trial."""
        self.reset["perms"]()
        self.reset["rules"]()
        self.reset["objects"]()
        self.reset["agents"]()
        self.reset["simulation"]()
    
    def testRuleLearning(self, n_iters):
        """Test rule learning by providing labelled examples."""

        # Fraction of objects whose owner labels are given
        own_frac = rospy.get_param("~own_frac", 1.0)
        # Average ownership probability for the training examples
        own_mean = rospy.get_param("~own_mean", 1.0)
        # Half-range of ownership probability for the training examples
        own_dev = rospy.get_param("~own_dev", 0.0)
        # Fraction of owner-labeled objects for which permissions are given
        perm_frac = rospy.get_param("~perm_frac", 1.0)

        # Disable ownership inference and extrapolation
        self.disable["inference"](True)
        self.disable["extrapolate"](True)
        
        tot_acc = 0.0
        for i in range(n_iters):
            print "-- Trial {} --".format(i+1)
            self.resetAll()
            rospy.sleep(self.iter_wait)
            self.introduceAgents()
            self.loadRules()
            objs = self.introduceOwners(own_frac, own_mean, own_dev)
            self.batchPermInstruct(perm_frac, objs)
            trial_acc, rule_acc = rule_instructor.evaluateRules()
            tot_acc += trial_acc
            
        tot_acc /= n_iters
        print "Total accuracy over {} trials: {}".format(n_iters, tot_acc)
        return tot_acc
            
if __name__ == '__main__':
    rospy.init_node('rule_instructor')
    rule_instructor = RuleInstructor()
    if rospy.get_param("~online", False):
        rule_instructor.initOnline()
        rospy.spin()
    else:
        n_iters = rospy.get_param("~n_iters", 1)
        rule_instructor.testRuleLearning(n_iters)
        
