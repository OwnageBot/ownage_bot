#!/usr/bin/env python
import rospy
import threading
import Queue
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
        self.q_lock = threading.Lock()
        self.action_queue = Queue.Queue()

        # Subscribers and publishers
        self.command_sub = rospy.Subscriber("command", TaskMsg, self.commandCb)
        self.feedback_pub = rospy.Publisher("feedback", FeedbackMsg,
                                            queue_size=10)

        # Look up clients for object permissions, and rules
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        self.lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)
        self.lookupPerm = rospy.ServiceProxy("lookup_perm", LookupPerm)
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

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
            else:
                tgt = action.tgtype.fromStr(cmd.target)
                task = Task.oneShot(action, tgt)
        elif cmd.name in self.task_db:
            task = self.task_db[cmd.name]
        self.cur_task = task
        # Cancel current action and empty action queue if interrupt is true
        if cmd.interrupt:
            actions.Cancel.call()
            self.q_lock.acquire()
            while self.action_queue.qsize() > 0:
                self.action_queue.get(False)
            self.q_lock.release()

    def updateActions(self):
        "Updates actions based on world state."
        try:
            rospy.wait_for_service("list_objects", 0.5)
            resp = self.listObjects()
        except rospy.ROSException:
            rospy.logwarn("list_objects service not available")
            return
        olist = [Object.fromMsg(msg) for msg in resp.objects]
        object_db = dict(zip([o.id for o in olist], olist))
        self.q_lock.acquire()
        self.cur_task.updateActions(self.action_queue, object_db)
        self.q_lock.release()

    def checkPerms(self, action, tgt):
        """Returns true if action on specific target is forbidden."""
        for a in (action.dependencies + [action]):
            perm = self.lookupPerm(action.name, tgt.toStr()).perm
            if perm >= 0.5:
                return True
        return False

    def checkRules(self, action, tgt):
        """Returns true if rules forbid action on target."""
        for a in (action.dependencies + [action]):
            rule_set = self.lookupRules(a.name).rule_set
            rule_set = [Rule.fromMsg(r) for r in rule_set]
            if Rule.evaluateOr(rule_set, tgt) >= 0.5:
                return True
        return False
    
    def main(self):
        """Main loop which manages tasks and responds to commands."""
        
        # Wait for other nodes to start, then go home
        rospy.wait_for_service("/action_provider/service_left")
        actions.GoHome.call()

        # Keep performing requested tasks/actions
        while not rospy.is_shutdown():
            # Update action queue for current task
            self.updateActions()
            # Get next action-target pair
            self.q_lock.acquire()
            if self.action_queue.qsize() > 0:
                action, tgt = self.action_queue.get(True, 0.5)
                self.q_lock.release()
            else:
                self.q_lock.release()
                continue
            if isinstance(tgt, Object):
                # Get most recent information about object
                tgt = Object.fromID(tgt.id)
            # Check if action still needs to be done
            if self.cur_task.checkActionDone(action, tgt):
                continue
            # Check if action is forbidden by permissions or rules
            if self.checkPerm(action, tgt) or self.checkRules(action, tgt):
                rospy.logwarn("%s on %s is forbidden",
                              action.name, tgt.toStr())
                continue
            # Call action if all checks pass
            resp = action.call(tgt)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()
    rospy.spin()
