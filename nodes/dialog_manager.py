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
        # Databases of available actions and tasks
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))
        self.task_db = dict(zip([t.name for t in tasks.db], tasks.db))
        
        # Subscribers and publishers
        self.input_sub = rospy.Subscriber("dialog_in", String, self.inputCb)
        self.output_pub = rospy.Publisher("dialog_out", String, queue_size=10)
        self.command_pub = rospy.Publisher("command", TaskMsg, queue_size=10)

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
        else:
            pass
        if name == tasks.Idle.name:
            print "Could not parse input, defaulting to idle task."
        
if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
