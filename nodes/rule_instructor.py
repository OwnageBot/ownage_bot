#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class RuleInstructor:
    """Automatically teaches rules by exampxle or direct instruction."""

    def __init__(self):
        # Load rules from config file
        self.rule_db = []
        
        # Publishers
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=100)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=100)

        # Servers
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        
    def initOnline(self):
        """Sets up subscribers for online instruction."""
        pass
    
    def batchInstruct(self):
        """Teaches all information in one batch."""
        mode = rospy.get_param("~mode", "perm")
        if mode == "perm":
            rospy.wait_for_service("list_objects")
            objs = [o.fromMsg for o in self.listObjects().objects]
            for r in self.rule_db:
                for o in objs:
                    truth = r.evaluate(o)
                    perm = PredicateMsg(predicate=r.action.name,
                                        bindings=[o.toStr()],
                                        truth=truth)
                    self.perm_pub.publish(perm)
        elif mode == "rule":
            for r in self.rule_db:
                self.rule_pub.publish(r.toMsg())
        elif mode == "script":
            for msg in self.script_msgs:
                if isinstance(msg, PredicateMsg):
                    self.perm_pub.publish(msg)
                elif isinstance(msg, RuleMsg):
                    self.rule_pub.publish(msg)
        else:
            rospy.logerror("Instructor mode %s unrecognized", mode)
            
if __name__ == '__main__':
    rospy.init_node('rule_instructor')
    rule_instructor = RuleInstuctor()
    if rospy.get_param("~online", False):
        rule_instructor.initOnline()
    else:
        rule_instructor.batchInstruct()
    rospy.spin()
        
