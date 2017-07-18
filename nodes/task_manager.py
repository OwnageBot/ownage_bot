#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from human_robot_collaboration_msgs.srv import DoAction
from human_robot_collaboration_msgs.srv import DoActionResponse
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

ACT_CANCELLED = "Action cancelled by user"

class TaskManager:
    """Manages the task currently assigned to the robot."""

    def __init__(self):
        # Rectangle denoting home area
        if rospy.has_param("home_area"):
            self.home_area = (Point(*rospy.get_param("home_area/lower")),
                              Point(*rospy.get_param("home_area/upper")))
        else:
            self.home_area = (Point(0.39,0.07, 0), Point(0.62, 0.29, 0))
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.pickerProxy = rospy.ServiceProxy(
            "/action_provider/service_left", DoAction)
        self.classifyProxy = rospy.ServiceProxy(
            "classify_objects", ListObjects)
        self.feedback_pub = rospy.Publisher("feedback", FeedbackMsg,
                                             queue_size = 10)

    # Python wrappers for the actions defined in object_picker.h
    def scanWorkspace(self):
        return self.pickerProxy("scan", [])

    def goHome(self):
        return self.pickerProxy("home", [])

    def pickUp(self, obj):
        return self.pickerProxy("get", [obj.id])

    def putDown(self):
        return self.pickerProxy("put", [])

    def find(self, obj):
        return self.pickerProxy("find", [obj.id])

    def offer(self, a):
        return self.pickerProxy("offer", [a])

    def waitForFeedback(self):
        return self.pickerProxy("wait", [])

    def replace(self):
        return self.pickerProxy("replace", [])

    def collect(self, obj):
        """Attempts to bring object to home area.

        Returns 0 on success (object is unowned).
        Returns avatar id of owner if object is claimed.
        Returns -1 if some other error occurs.
        """
        if not self.find(obj).success:
            print("Failed finding obj!\n")
            return -1
        if not self.pickUp(obj).success:
            print("Failed picking up object!\n")
            return -1
        ret = self.goHome()
        if not ret.success:
            if ret.response == ACT_CANCELLED:
                # Sleep to prevent double cancellation
                rospy.sleep(4)
                if not self.goHome().success:
                    print("Failed going home!\n")
                    self.replace()
                    return -1
                return self.offerInTurn(obj)
            else:
                print("Failed going home!\n")
                return -1
        ret = self.waitForFeedback()
        if not ret.success:
            if ret.response == ACT_CANCELLED:
                # Sleep to prevent double cancellation
                rospy.sleep(4)
                if not self.goHome().success:
                    print("Failed going home!\n")
                    self.replace()
                    return -1
                return self.offerInTurn(obj)
            else:
                print("Failed waiting for feedback!\n")
                return -1
        if not self.putDown().success:
            print("Failed putting down object!\n")
            return -1
        return 0

    def inHomeArea(self, obj):
        """Checks if object is in home area."""
        loc = obj.pose.pose.position
        return ((self.home_area[0].x <= loc.x <= self.home_area[1].x) and
                (self.home_area[0].y <= loc.y <= self.home_area[1].y))

    def feedback(self, obj, label):
        """Gives label feedback to classifier."""
        msg = FeedbackMsg()
        msg.stamp = rospy.Time.now()
        msg.label = label
        msg.object = obj
        self.feedback_pub.publish(msg)

    def inputCallback(self, data)
        """Handles incoming text input."""
        command, feedback, interrupt = self.parseInput(data)
        if interrupt:
            self.cancel()
        self.sendFeedback(feedback)
        if self.evaluateCommand(command):
            self.actionQueue.append(command.action)

    def main(self):
        """Main loop which manages tasks and responds to commands."""
        
        # Wait for other nodes to start, then go home
        rospy.wait_for_service("/action_provider/service_left")
        self.goHome()

        # Keep scanning for objects until shutdown
        while not rospy.is_shutdown():
            action = self.actionQueue.popleft()
            resp = self.callAction(action)

if __name__ == '__main__':
    rospy.init_node('task_manager')
    task_manager = TaskManager()
    task_manager.main()
    rospy.spin()

