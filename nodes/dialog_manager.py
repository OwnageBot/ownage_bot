#!/usr/bin/env python
import rospy
import re
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager:
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Databases of available actions and tasks
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))
        self.task_db = dict(zip([t.name for t in tasks.db], tasks.db))

        # Handle input and output to/from users
        self.input_sub = rospy.Subscriber("dialog_in", String, self.inputCb)
        self.output_pub = rospy.Publisher("dialog_out", String, queue_size=10)

        self.command_pub = rospy.Publisher("command", TaskMsg, queue_size=10)
        self.fact_pub = rospy.Publisher("fact_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)
        
    def inputCb(self, msg):
        """Handles dialog input and publishes commands."""
        name = tasks.Idle.name

        args = msg.data.split()

        if len(args) == 0:
            return
        if args[0] == "list":
            # List available actions
            out = "\n".join(["Available actions:"] + self.action_db.keys())
            self.output_pub.publish(out)
            return
        elif args[0] in self.action_db:
            # Try one of the higher-level tasks
            name = args[0]
            tgt = "" if len(args) == 1 else args[1]
            cmd = TaskMsg(name=name, oneshot=True, interrupt=True, target=tgt)
            self.command_pub.publish(cmd)
            return
        elif args[0] in self.task_db:
            # Try one of the higher-level tasks
            name = args[0]
            cmd = TaskMsg(name=name, oneshot=False, interrupt=True, target="")
            self.command_pub.publish(cmd)
            return

        # Try to match facts
        match = re.match("(forbid|allow) (\S+) on (\d*)", msg.data)
        if match:
            action = match.group(2)
            tgt = match.group(3)
            truth = float(match.group(1) == "forbid")
            fact = PredicateMsg(predicate=action, bindings=[tgt],
                                negated=False, truth=truth)
            self.output_pub.publish(str(fact))
            self.fact_pub.publish(fact)
            return

        # Try to match rules
        match = re.match("(forbid|allow) (\S+) if (.+)", msg.data)
        if match:
            action = match.group(2)
            truth = float(match.group(1) == "forbid")
            pred_strs = match.group(3).strip().split(" and ")
            conditions = []
            for s in pred_strs:
                pred_args = s.split()
                predicate = pred_args[0] # Extract predicate name
                pred_args[0] = "" # Leave first argument unbound
                pred_msg = PredicateMsg(predicate, pred_args, False, 1.0)
                conditions.append(pred_msg)
            rule = RuleMsg(action, conditions, "forbidden", truth)
            self.output_pub.publish(str(rule))
            self.rule_pub.publish(rule)
            return
        
        if name == tasks.Idle.name:
            print "Could not parse input, defaulting to idle task."
        
if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
