#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from human_robot_collaboration_msgs.srv import DoAction, DoActionResponse
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

ACT_CANCELLED = "Action cancelled by user"

class ObjectCollector:
    """A class for collecting objects."""

    def __init__(self):
        self.skipScan = False
        # Threshold for object collection
        self.threshold = (rospy.get_param("collect_threshold") if
                          rospy.has_param("collect_threshold") else 0.2)
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

    def offerInTurn(self, obj):
        """Offers held object to each avatar in turn.

        Returns 0 if all avatars reject object (object is unowned).
        Returns avatar id of owner if object is claimed.
        Returns -1 if some fatal error occurs.
        """
        for a in self.avatar_ids:
            ret = self.offer(a)
            if ret.success:
                # If offer is successful, replace object
                self.replace()
                return a
            elif ret.response == ACT_CANCELLED:
                # Continue on to next possible owner if rejected
                print("Object {} rejected by avatar {}\n".
                      format(obj.id, a))
                rospy.sleep(3)
            else:
                # Error out upon some other kind of failure
                print("Failed to offer avatar {} object {}!\n".
                      format(a, obj.id))
            # Return to home position before making next offer
            self.goHome()
            print("Continuing to next avatar...\n")
        # Claim object for self and return 0 if everyone else rejects it
        if not self.putDown().success:
            print("Failed putting down object!\n")
            return -1
        return 0

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
        loc = obj.position
        return ((self.home_area[0].x <= loc.x <= self.home_area[1].x) and
                (self.home_area[0].y <= loc.y <= self.home_area[1].y))

    def classify(self):
        """Requests ObjectClassifier to determine if objects are owned."""
        try:
            resp = self.classifyProxy()
            return resp.objects
        except rospy.service.ServiceException:
            print("ListObject service call failed!\n")
            return []

    def feedback(self, obj, label):
        """Gives label feedback to classifier."""
        msg = FeedbackMsg()
        msg.stamp = rospy.Time.now()
        msg.label = label
        msg.object = obj
        self.feedback_pub.publish(msg)

    def main(self):
        """Main loop which collects all tracked objects."""
        
        # Wait for other nodes to start, then go home
        rospy.wait_for_service("/action_provider/service_left")
        self.goHome()

        # Keep scanning for objects until shutdown
        while not rospy.is_shutdown():
            if self.skipScan:
                self.skipScan = False
            else:
                self.goHome()
                self.scanWorkspace()
                
            # Get list of classified objects and sort by ownership probability
            objects = self.classify()
            objects.sort(key=lambda o: o.ownership[0], reverse=True)
            for obj in objects:
                # Pick up only if object is outside home area and unowned
                if (obj.is_avatar or self.inHomeArea(obj) or
                    obj.ownership[0] <= self.threshold):
                    continue
                rospy.loginfo("Collecting object {}\n".format(obj.id))
                label = self.collect(obj)
                # Continue to next object if there was an error in collection
                if label == -1:
                    rospy.loginfo("Error collecting object {}\n".
                                  format(obj.id))
                    continue
                # Update ownership rules and reclassify if no fatal error
                if label > 0:
                    rospy.loginfo("Object {} was labeled as {}\n".
                                  format(obj.id, label))
                else:
                    rospy.loginfo("Object {} was unclaimed\n".
                                  format(obj.id))
                self.feedback(obj, label)
                self.skipScan = True
                break

if __name__ == '__main__':
    rospy.init_node('object_collector')
    objectCollector = ObjectCollector()
    objectCollector.main()
    rospy.spin()

