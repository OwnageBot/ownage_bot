#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleInstructor:
    """Automatically teaches rules by exampxle or direct instruction."""

    def __init__(self):
        self.mode = rospy.get_param("~mode", "by_perm")

        if self.mode == "by_script":
            self.script_msgs = self.loadScript()
        else:
            self.rule_db = self.loadRules()
            
        self.pub_rate = rospy.Rate(rospy.get_param("~pub_rate", 10))
            
        # Publishers
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)

        # Servers
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)

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

    def batchPermInstruct(self):
        """Provides all object-specific permissions in one batch."""
        objs = list(Object.universe())
        # Randomize object order
        random.shuffle(objs)
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
            if isinstance(msg, PredicateMsg):
                self.perm_pub.publish(msg)
            elif isinstance(msg, RuleMsg):
                self.rule_pub.publish(msg)
            self.pub_rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('rule_instructor')
    rule_instructor = RuleInstructor()
    if rospy.get_param("~online", False):
        rule_instructor.initOnline()
        rospy.spin()
    else:
        rule_instructor.batchInstruct()
        
