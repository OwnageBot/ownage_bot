#!/usr/bin/env python
import rospy
import Queue
from std_msgs.msg import UInt32
from std_msgs.msg import String
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

class TaskManager:
    """Manages the task currently assigned to the robot."""

    def __init__(self):
        # Duration in seconds between action updates
        self.update_latency = rospy.get_param("task_update", 0.5)
        # Current task being performed
        self.current_task = tasks.Idle
        # Database of rules to follow
        self.rule_db = [rules.DoNotTouchRed, rules.DoNotTrashBlue]
        # Database of available actions
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))
        # Queue of action-target pairs
        self.action_queue = Queue.Queue()
        self.avatar_ids = rospy.get_param("avatar_ids", [])
        self.input_sub = rospy.Subscriber("text_input", String, self.inputCb)
        self.feedback_pub = rospy.Publisher("feedback", FeedbackMsg,
                                            queue_size = 10)
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        self.lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)

    def sendFeedback(self, feedback):
        """Gives feedback for rule learning."""
        msg = FeedbackMsg()
        msg.stamp = rospy.Time.now()
        self.feedback_pub.publish(msg)

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
                    obj = Object(self.lookupObject(oid).object)
                    task = Task.oneShot(action, obj)
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
        if interrupt:
            actions.Cancel.call()
        self.sendFeedback(feedback)
        if not task is None:
            self.current_task = task

    def updateCb(self, event):
        "Callback that updates actions based on world state."
        try:
            rospy.wait_for_service("list_objects", 0.5)
            resp = self.listObjects()
        except rospy.ROSException:
            rospy.logwarn("list_objects service not available")
            return
        olist = [Object(msg) for msg in resp.objects]
        object_db = dict(zip([o.id for o in olist], olist))
        self.current_task.updateActions(self.action_queue, object_db)

    def main(self):
        """Main loop which manages tasks and responds to commands."""
        
        # Wait for other nodes to start, then go home
        rospy.wait_for_service("/action_provider/service_left")
        actions.GoHome.call()

        # Periodically update actions based on world state
        rospy.Timer(rospy.Duration(self.update_latency), self.updateCb)

        # Keep performing requested tasks/actions
        while not rospy.is_shutdown():
            # Get next action-target pair
            try:
                action, tgt = self.action_queue.get(True, 0.5)
            except Queue.Empty:
                continue
            # Evaluate all rules applicable to current action
            for rule in self.rule_db:
                if action != rule.action:
                    continue
                if rule.detype == Rule.forbidden and rule.evaluate(tgt):
                    break;
                elif rule.detype == Rule.allowed and not rule.evaluate(tgt):
                    break;
            else:
                # Call action if rules allow for it
                resp = action.call(tgt)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()
    rospy.spin()
