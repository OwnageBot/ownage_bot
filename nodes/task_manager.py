#!/usr/bin/env python
import rospy
from Queue import Queue
from std_msgs.msg import UInt32
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

ACT_CANCELLED = "Action cancelled by user"

class TaskManager:
    """Manages the task currently assigned to the robot."""

    def __init__(self):
        # Duration in seconds between action updates
        self.update_latency = 0.5
        # Current task being performed
        self.current_task = None
        # Database of rules to follow
        self.rule_db = [rules.DoNotTouchRed, rules.DoNotTrashBlue]
        # Queue of actions to be performed
        self.action_queue = Queue()
        # Rectangle denoting home area
        if rospy.has_param("home_area"):
            self.home_area = (Point(*rospy.get_param("home_area/lower")),
                              Point(*rospy.get_param("home_area/upper")))
        else:
            self.home_area = (Point(0.39,0.07, 0), Point(0.62, 0.29, 0))
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.feedback_pub = rospy.Publisher("feedback", FeedbackMsg,
                                             queue_size = 10)
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)

    def sendFeedback(self, feedback):
        """Gives feedback for rule learning."""
        msg = FeedbackMsg()
        msg.stamp = rospy.Time.now()
        self.feedback_pub.publish(msg)

    def parseInput(self, data):
        "Parses text input and returns commands."
        task = None
        if data == "tidy":
            task = tasks.Tidy
        elif data == "trash":
            task = tasks.Trash
        feedback = None
        interrupt = True
        return task, feedback, interrupt

    def inputCb(self, data):
        """Handles incoming text input."""
        task, feedback, interrupt = self.parseInput(data)
        if interrupt:
            actions.Cancel.call()
        self.sendFeedback(feedback)
        if not task is None:
            self.current_task = task

    def updateCb(self):
        "Callback that updates actions based on world state."
        resp = self.listObjects()
        object_db = dict(zip([o.id for o in resp.objects], resp.objects))
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
            # Get next action-object pair, blocks until one is available
            action, obj = self.action_queue.get(True)
            # Evaluate all rules applicable to current action
            for rule in self.rule_db:
                if action != rule.action:
                    continue
                if rule.detype == Rule.forbidden and rule.evaluate(obj):
                    break;
                elif rule.detype == Rule.allowed and not rule.evaluate(obj):
                    break;
            else:
                # Call action if rules allow for it
                resp = action.call(obj)
                # TODO: Use response from action as feedback?

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()

