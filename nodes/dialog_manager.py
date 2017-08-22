#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager:
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Handle input and output to/from users
        self.input_sub = rospy.Subscriber("dialog_in", String, self.inputCb)
        self.output_pub = rospy.Publisher("dialog_out", String, queue_size=10)

        self.command_pub = rospy.Publisher("command", TaskMsg, queue_size=10)
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)
        
    def inputCb(self, msg):
        """Handles dialog input and publishes commands."""
        args = msg.data.split()
        
        if len(args) == 0:
            return
        # List available actions
        if args[0] == "list":
            out = "\n".join(["Available actions:"] + actions.db.keys())
            self.output_pub.publish(out)
            return
        # Try parsing a one-shot task (i.e. an action)
        task = parse.asAction(msg.data)
        if task:
            self.command_pub.publish(task)
            return
        # Try parsing a higher level task
        task = parse.asTask(msg.data)
        if task:
            self.command_pub.publish(task)
            return
        # Try to parse object-specific permissions
        perm = parse.asPerm(msg.data)
        if perm:
            self.output_pub.publish(str(perm))
            self.perm_pub.publish(perm)
            return
        # Try to parse rules that govern actions
        rule = parse.asRule(msg.data)
        if rule:
            self.output_pub.publish(str(rule))
            self.rule_pub.publish(rule)
            return

        # Error out if nothing works
        out = "Could not parse input, defaulting to idle task."
        self.output_pub.publish(out)
        
if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
