#!/usr/bin/env python
import rospy
from baxter_core_msgs.msg import EndpointState, EndEffectorState
from human_robot_collaboration_msgs.msg import ArmState
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *
from object_tracker import ObjectTracker

class EndpointTracker(ObjectTracker):
    """Tracks held objects using endpoint location."""

    def __init__(self):
        super(EndpointTracker, self).__init__()

        # How frequently position is updated
        self.endpoint_latency = rospy.get_param("~endpoint_latency", 0.1)
        self.endpoint_update_t = rospy.get_rostime()
        
        # State variables to track gripped objects
        self.cur_action = ""
        self.prev_action = ""
        self.gripped_id = -1
        
        # Subscribe to action state
        self.action_sub = rospy.Subscriber("/action_provider/left/state",
                                           ArmState, self.actionCb)
        self.endpoint_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",
                                             EndpointState, self.endpointCb)

    def actionCb(self, msg):
        """Callback upon change in current action."""
        self.prev_action = self.cur_action
        self.cur_action = msg.action
        if self.cur_action == self.prev_action:
            return
        if self.cur_action == "pickUp" and int(msg.object) in self.object_db:
            # Start tracking object at endpoint
            self.gripped_id = int(msg.object)
        elif self.cur_action in ["putDown", "release"]:
            # Stop tracking object at endpoint
            self.gripped_id = -1

    def endpointCb(self, msg):
        """Callback for endpoint state, used to track gripped objects."""
        # Only update if object is gripped
        if self.gripped_id < 0:
            return
        gripped_id = self.gripped_id

        # Don't update too frequently
        t_now = rospy.get_rostime()
        t_diff = (t_now - self.endpoint_update_t).to_sec()
        if t_diff <= self.endpoint_latency:
            return
        self.endpoint_update_t = t_now
       
        # Check if gripper is still gripping
        topic = "/robot/end_effector/left_gripper/state"
        state = rospy.wait_for_message(topic, EndEffectorState)
        if state.gripping:
            print msg.pose.position
            # Update gripped object's position in place
            self.object_db[gripped_id].position = msg.pose.position

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    EndpointTracker()
    rospy.spin()
