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
        self.cur_task = tasks.Idle
        # Database of rules to follow
        self.rule_db = [rules.DoNotTouchRed, rules.DoNotTrashBlue]
        # Database of available actions
        self.action_db = dict(zip([a.name for a in actions.db], actions.db))
        # Database of available tasks
        self.task_db = dict(zip([t.name for t in tasks.db], tasks.db))

        # Queue of action-target pairs
        self.action_queue = Queue.Queue()
        self.avatar_ids = rospy.get_param("avatar_ids", [])
        self.command_sub = rospy.Subscriber("command", TaskMsg, self.commandCb)
        self.feedback_pub = rospy.Publisher("feedback", FeedbackMsg,
                                            queue_size=10)
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        self.lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)

    def sendFeedback(self, feedback):
        """Gives feedback for rule learning."""
        msg = FeedbackMsg()
        msg.stamp = rospy.Time.now()
        self.feedback_pub.publish(msg)

    def commandCb(self, cmd):
        """Handles incoming commands."""
        # Determine and set new task
        task = tasks.Idle
        if cmd.oneshot:
            action = self.action_db[cmd.name]
            if action.tgtype is type(None):
                task = Task.oneShot(action, None)
            elif action.tgtype == Object:
                obj = Object.fromID(cmd.obj_id)
                task = Task.oneShot(action, obj)
            elif action.tgtype == Point:
                task = Task.oneShot(action, cmd.location)
        elif cmd.name in self.task_db:
            task = self.task_db[cmd.name]
        self.cur_task = task
        # Cancel current action and empty action queue if interrupt is true
        if cmd.interrupt:
            actions.Cancel.call()
            while self.action_queue.qsize() > 0:
                try:
                    self.action_queue.get(False)
                except Queue.Empty:
                    continue

    def updateCb(self, event):
        "Callback that updates actions based on world state."
        try:
            rospy.wait_for_service("list_objects", 0.5)
            resp = self.listObjects()
        except rospy.ROSException:
            rospy.logwarn("list_objects service not available")
            return
        olist = [Object.fromMsg(msg) for msg in resp.objects]
        object_db = dict(zip([o.id for o in olist], olist))
        self.cur_task.updateActions(self.action_queue, object_db)

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
            # Check if action still needs to be done
            if isinstance(tgt, Object):
                # Get most recent information about object
                tgt = Object.fromID(tgt.id)
            if not self.cur_task.checkActionUndone(action, tgt):
                continue
            # Evaluate all rules applicable to current action
            for rule in self.rule_db:
                if rule.action not in ([action] + action.dependencies):
                    continue
                if rule.detype == Rule.forbidden and rule.evaluate(tgt):
                    print "Cannot violate rule:", rule.toStr()
                    break;
                elif rule.detype == Rule.allowed and not rule.evaluate(tgt):
                    print "Cannot violate rule:", rule.toStr()
                    break;
            else:
                # Call action if rules allow for it
                resp = action.call(tgt)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()
    rospy.spin()
