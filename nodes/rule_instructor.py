#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleInstructor(object):
    """Automatically teaches rules by example or direct instruction."""

    def __init__(self):
        # Learning mode (by rule, permission, or script)
        self.mode = rospy.get_param("~mode", "by_perm")
        # Fraction of objects used as training examples
        self.train_frac = rospy.get_param("~train_frac", 0.5)
        # Fraction of objects whose owners are introduced before training
        self.intro_frac_pre = rospy.get_param("~intro_frac_pre", 0.0)
        # Fraction of objects whose owners are introduced after training
        self.intro_frac_post = rospy.get_param("~intro_frac_post", 0.0)

        self.agt_pub_rate = rospy.Rate(rospy.get_param("~agt_pub_rate", 3))
        self.pub_rate = rospy.Rate(rospy.get_param("~pub_rate", 10))
            
        # Publishers
        self.agent_pub = rospy.Publisher("agent_input", AgentMsg,
                                         queue_size=10)
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

    def loadInstructions(self):
        """Load instructions (should be called after introducing agents)."""
        if self.mode == "by_script":
            self.script_msgs = self.loadScript()
        else:
            self.rule_db = self.loadRules()
        
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
        return rule_db

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
        return script_msgs
        
    def initOnline(self):
        """Sets up subscribers for online instruction."""
        pass

    def introduceAgents(self):
        """Looks up all simulated agents and introduces them to the tracker."""
        agts = self.getAgents().agents
        for a in agts:
            self.agt_pub_rate.sleep()
            self.agent_pub.publish(a)

    def introduceOwners(self, fraction=1.0):
        """Provides ownership labels for a random subset of the objects."""
        objs = [Object.fromMsg(m) for m in self.getObjects().objects]
        sample = random.sample(objs, int(fraction * len(objs)))
        for o in sample:
            for agent_id, p_owned in o.ownership.iteritems():
                msg = PredicateMsg(predicate=predicates.OwnedBy.name,
                                   bindings=[o.toStr(), str(agent_id)],
                                   negated=False, truth=p_owned)
                self.pub_rate.sleep()
                self.owner_pub.publish(msg)
            
    def batchInstruct(self):
        """Teaches all information in one batch."""
        if self.intro_frac_pre > 0:
            rule_instructor.introduceOwners(self.intro_frac_pre)
        if self.mode == "by_perm":
            self.batchPermInstruct()
        elif self.mode == "by_rule":
            self.batchRuleInstruct()
        elif self.mode == "by_script":
            self.batchScriptInstruct()
        else:
            rospy.logerror("Instructor mode %s unrecognized", mode)
        if self.intro_frac_post > 0:
            rule_instructor.introduceOwners(self.intro_frac_post)
            
    def batchPermInstruct(self):
        """Provides all object-specific permissions in one batch."""
        objs = [Object.fromMsg(m) for m in self.getObjects().objects]
        # Get rid of avatars
        objs = [o for o in objs if not o.is_avatar]
        # Randomize object order
        random.shuffle(objs)
        # Use only a fraction as training examples (rounded down)
        objs = objs[0:int(self.train_frac * len(objs))]
        # Feed permissions for each action separately
        for act_name, rule_set in self.rule_db.iteritems():
            for o in objs:
                if o.is_avatar:
                    continue
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
            
if __name__ == '__main__':
    rospy.init_node('rule_instructor')
    rule_instructor = RuleInstructor()
    if rospy.get_param("~online", False):
        rule_instructor.initOnline()
        rospy.spin()
    else:
        rule_instructor.introduceAgents()
        rule_instructor.loadInstructions()
        rule_instructor.batchInstruct()
        
