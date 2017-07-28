#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *

class DialogManager:
    """Manages dialog with users, generates replies and parses instructions."""

    def __init__(self):
        # Database of available actions
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))        
        
        # Subscribers and publishers
        self.input_sub = rospy.Subscriber("text_input", String, self.inputCb)
        self.command_pub = rospy.Publisher("command", Command, queue_size = 10)

    def parseInput(self, data):
        "Parses text input and returns commands."
        task = tasks.Idle
        feedback = None
        interrupt = True

        args = data.split()

        if len(args) == 0:
            return task, feedback, interrupt
        if args[0] == "list":
            # List available actions
            print "Available actions:"
            for a in self.action_db.iterkeys():
                print a
            return task, feedback, interrupt
        elif args[0] in self.action_db:
            # Construct one-shot task if syntax matches
            action = self.action_db[args[0]]
            if action.tgtype is type(None):
                task = Task.oneShot(action, None)
            elif len(args) >= 2:
                if action.tgtype == Object:
                    oid = int(args[1])
                    task = Task.oneShot(action, oid)
                elif action.tgtype == Point:
                    loc = Point(*[float(s) for s in args[1].split(',')])
                    task = Task.oneShot(action, loc)
        else:
            # Try one of the higher-level tasks
            if args[0] == "collectAll":
                task = tasks.CollectAll
            elif args[0] == "trashAll":
                task = tasks.TrashAll
        if task == tasks.Idle:
            print "Could not parse input, defaulting to idle task."
        return task, feedback, interrupt

    def inputCb(self, msg):
        """Handles incoming text input."""
        task, feedback, interrupt = self.parseInput(msg.data)

if __name__ == '__main__':
    rospy.init_node('dialog_manager')
    rospy.spin()
