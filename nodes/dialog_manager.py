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
        self.input_sub = rospy.Subscriber("text_input", String, self.inputCb)
        self.command_pub = rospy.Publisher("command", TaskMsg, queue_size=10)

    def inputCb(self, msg):
        "Handles text input and publishes commands."
        name = tasks.Idle.name
        obj_id = -1
        location = Point()

        args = msg.data.split()
        if len(args) == 0:
            return
        if args[0] == "list":
            # List available actions
            print "Available actions:"
            for a in self.action_db.iterkeys():
                print a
            return
        elif args[0] in self.action_db:
            # Send one-shot command if syntax matches
            name = args[0]
            action = self.action_db[name]
            if len(args) >= 2:
                if action.tgtype == Object:
                    obj_id = int(args[1])
                elif action.tgtype == Point:
                    location = Point(*[float(s) for s in args[1].split(',')])
            cmd = TaskMsg(name=name, oneshot=True, interrupt=True,
                          obj_id=obj_id, location=location)
            self.command_pub.publish(cmd)
            return
        elif args[0] in self.task_db:
            # Try one of the higher-level tasks
            name = args[0]
            cmd = TaskMsg(name=name, oneshot=False, interrupt=True,
                          obj_id=obj_id, location=location)
            self.command_pub.publish(cmd)
            return
        if name == tasks.Idle.name:
            print "Could not parse input, defaulting to idle task."

if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    dialog_manager = DialogManager()
    rospy.spin()
