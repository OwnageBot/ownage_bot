#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager:
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Handle input and output to/from users
        self.input_sub = rospy.Subscriber("dialog_in", String,
                                          self.inputCb)
        self.output_pub = rospy.Publisher("dialog_out", String,
                                          queue_size=10)

        # Handle input and output from task manager node
        self.task_out_sub = rospy.Subscriber("task_out", String,
                                             self.taskOutCb)
        self.task_in_pub = rospy.Publisher("task_in", TaskMsg,
                                           queue_size=10)

        # Sends instructions to rule manager node
        self.perm_pub = rospy.Publisher("perm_input", PredicateMsg,
                                        queue_size=10)
        self.rule_pub = rospy.Publisher("rule_input", RuleMsg,
                                        queue_size=10)

        # Looks up rule database from rule manager
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

        # Resets various databases
        self.resetPerms = rospy.ServiceProxy("reset_perms", Trigger)
        self.resetRules = rospy.ServiceProxy("reset_rules", Trigger)
        self.resetWorld = rospy.ServiceProxy("reset_world", Trigger)
        
    def inputCb(self, msg):
        """Handles dialog input and publishes commands."""
        args = msg.data.split()
        
        if len(args) == 0:
            return
        # List various entities (actions, rules, etc.)
        if args[0] == "list":
            self.handleList(args)
            return
        # Reset various entities (rules, permissions, simulation)
        if args[0] == "reset":
            self.handleReset(args)
            return
        # Try parsing a one-shot task (i.e. an action)
        task = parse.asAction(msg.data)
        if task:
            self.task_in_pub.publish(task)
            return
        # Try parsing a higher level task
        task = parse.asTask(msg.data)
        if task:
            self.task_in_pub.publish(task)
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
        out = "Could not parse input."
        self.output_pub.publish(out)

    def taskOutCb(self, msg):
        """Handles output from TaskManager node."""
        # Just print response for now
        self.output_pub.publish(msg)

    def handleList(self, args):
        """Handles list command."""
        if len(args) < 2:
            out = "\n".join(["List one of the following:"] +
                            ['objects', 'predicates', 'rules',
                             'actions', 'tasks'])
        elif args[1] == "objects":
            objs = sorted(list(Object.universe()), key=lambda o : o.id)
            obj_strs = ["{:3d} {:10} ({:04.2f},{:04.2f},{:04.2f})"\
                        .format(o.id, o.color,
                                o.position.x, o.position.y, o.position.z)
                        for o in objs]
            out = "\n".join(["Tracked objects:"] + obj_strs)
        elif args[1] == "predicates":
            out = "\n".join(["Available predicates:"] + predicates.db.keys())
        elif args[1] == "rules":
            act_names = args[2:]
            rule_msgs = []
            if len(act_names) == 0:
                act_names = actions.db.keys()
            for a in act_names:
                rule_msgs += self.lookupRules(a).rule_set
            rule_strs = [Rule.fromMsg(m).toPrint() for m in rule_msgs]
            out = "\n".join(["Active rules:"] + rule_strs)
        elif args[1] == "actions":
            out = "\n".join(["Available actions:"] + actions.db.keys())
        elif args[1] == "tasks":
            out = "\n".join(["Available tasks:"] + tasks.db.keys())
        else:
            out = "Keyword not recognized."
        self.output_pub.publish(out)

    def handleReset(self, args):
        """Handles reset command."""
        if len(args) < 2:
            out = "\n".join(["Reset one of the following:"] +
                            ['perms', 'rules', 'world', 'all'])
        elif args[1] == "perms":
            self.resetPerms()
            return
        elif args[1] == "rules":
            self.resetRules()
            return
        elif args[1] == "world":
            try:
                rospy.wait_for_service("reset_world", timeout=0.5)
                self.resetWorld()
                return
            except rospy.ROSException:
                out = "World simulator does not seem to be running."
                self.output_pub.publish(out)
        elif args[1] == "all":
            try:
                rospy.wait_for_service("reset_world", timeout=0.5)
                self.resetWorld()
            except rospy.ROSException:
                pass
            self.resetPerms()
            self.resetRules()
        else:
            out = "Keyword not recognized."
            self.output_pub.publish(out)

            
if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
