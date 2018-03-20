#!/usr/bin/env python
import rospy
import threading
import Queue
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from std_msgs.msg import String
from std_srvs.srv import *
from geometry_msgs.msg import Point

class TaskManager(object):
    """Manages the task currently assigned to the robot."""

    def __init__(self):
        # Duration in seconds between action updates
        self.update_latency = rospy.get_param("task_update", 0.5)
        # Current task, action, and target
        self.cur_task = tasks.Idle
        self.cur_action = actions.Empty
        self.cur_target = None

        # Queue of action-target pairs
        self.q_lock = threading.Lock()
        self.action_queue = Queue.Queue()

        # Subscribers and publishers
        self.task_in_sub = rospy.Subscriber("task_in", TaskMsg, self.taskInCb)
        self.task_out_pub = rospy.Publisher("task_out", String, queue_size=10)

        # Look up clients for object permissions, and rules
        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        self.lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)
        self.lookupPerm = rospy.ServiceProxy("lookup_perm", LookupPerm)
        self.lookupRules = rospy.ServiceProxy("lookup_rules", LookupRules)

        # Servers that handle lookup requests
        self.cur_tsk_srv = rospy.Service("cur_task", Trigger,
                                         self.curTaskCb)
        self.cur_act_srv = rospy.Service("cur_action", Trigger,
                                         self.curActionCb)
        self.cur_tgt_srv = rospy.Service("cur_target", Trigger, 
                                         self.curTargetCb)

    def curTaskCb(self, req):
        """Returns name of the current task."""
        return TriggerResponse(True, self.cur_task.name)

    def curActionCb(self, req):
        """Returns name of the current action."""
        return TriggerResponse(True, self.cur_action.name)

    def curTargetCb(self, req):
        """Returns name of the current target."""
        name = '_nil_' if self.cur_target is None else self.cur_target.toStr()
        return TriggerResponse(True, name)

    def taskInCb(self, msg):
        """Handles incoming tasks."""
        task = tasks.Idle
        # Cancel current action and clear action queue on interrupt
        if msg.interrupt:
            self.cur_task = task
            actions.Cancel.call()
            self.q_lock.acquire()
            while not self.action_queue.empty():
                self.action_queue.get(False)
            self.q_lock.release()
        # Construct one-shot task if necessary
        if msg.oneshot:
            action = actions.db[msg.name]
            if action.tgtype is type(None):
                task = Task.oneShot(action, None)
            else:
                tgt = action.tgtype.fromStr(msg.target)
                task = Task.oneShot(action, tgt)
        elif msg.name in tasks.db:
            task = tasks.db[msg.name]
        self.cur_task = task

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

    def checkPerm(self, action, tgt):
        """Returns true if action on specific target is forbidden."""
        for a in (action.dependencies + [action]):
            tgt_str = "" if action.tgtype is type(None) else tgt.toStr()
            perm = self.lookupPerm(a.name, tgt_str).perm
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

        # Periodically update actions based on world state
        rospy.Timer(rospy.Duration(self.update_latency),
                    lambda evt : self.updateActions())
        
        # Keep performing requested tasks/actions
        while not rospy.is_shutdown():
            # Get next action-target pair
            try:
                action, tgt = self.action_queue.get(block=True, timeout=0.5)
            except Queue.Empty:
                continue
            if isinstance(tgt, Object):
                # Get most recent information about object
                tgt = Object.fromID(tgt.id)
            # Check if action still needs to be done
            if self.cur_task.checkActionDone(action, tgt):
                continue
            # Check if action is forbidden by permissions or rules
            if self.checkPerm(action, tgt) or self.checkRules(action, tgt):
                out = "{} on {} is forbidden".\
                    format(action.name, tgt.toPrint())
                self.task_out_pub.publish(out)
                continue
            # Call action if all checks pass
            self.cur_action = action
            self.cur_target = tgt
            resp = action.call(tgt)
            if not resp.success:
                self.task_out_pub.publish(resp.response)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()
    rospy.spin()
