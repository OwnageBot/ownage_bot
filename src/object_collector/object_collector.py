#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from baxter_collaboration_msgs.srv import DoAction
from ownage_bot.msg import RichObject
from ownage_bot.msg import RichObjectArray
from ownage_bot.srv import ClassifyObjects
from geometry_msgs.msg import Point

DUMMY_OBJ = -1

class ObjectCollector:
    """A class for collecting objects."""

    def __init__(self):
        self.skipScan = False
        # Threshold for forbiddenness
        self.threshold = (rospy.get_param("collect_threshold") if
                          rospy.has_param("collect_threshold") else 0.8)
        # Rectangle denoting home area
        if rospy.has_param("home_area"):
            self.home_area = (Point(*rospy.get_param("home_area/lower")),
                              Point(*rospy.get_param("home_area/upper")))
        else:
            self.home_area = (Point(0.45,-0.2, 0), Point(0.55, 0.2, 0))
        self.actionProvider = rospy.ServiceProxy(
            "/action_provider/service_left", DoAction)
        self.objectClassifier = rospy.ServiceProxy(
            "classifyObjects", ClassifyObjects)
        self.blacklist_pub = rospy.Publisher("blacklist", RichObject,
                                             queue_size = 10)
        
    def scanWorkspace(self):
        return self.actionProvider("scan", [DUMMY_OBJ])

    def goHome(self):
        return self.actionProvider("home", [DUMMY_OBJ])

    def pickUp(self, obj):
        return self.actionProvider("get", [obj.id])

    def putDown(self):
        return self.actionProvider("put", [DUMMY_OBJ])
    
    def collect(self, obj):
        """Attempts to bring object to home area."""
        ret = self.pickUp(obj)
        if not ret.success:
            return ret
        ret = self.goHome()
        if not ret.success:
            return ret
        ret = self.putDown()
        if not ret.success:
            return ret
    
    def inHomeArea(self, obj):
        """Checks if object is in home area."""
        loc = obj.pose.pose.position
        return ((self.home_area[0].x <= loc.x <= self.home_area[1].x) and
                (self.home_area[0].y <= loc.y <= self.home_area[1].y))

    def classify(self, objects):
        """Requests ObjectClassifier to determine if objects are owned."""
        return self.objectClassifier(objects)

    def blacklist(self, obj):
        """Tells classifier to blacklist given object and update ownership."""
        self.blacklist_pub.publish(obj)
    
    def main(self):
        """Main loop which collects all tracked objects."""
        while not rospy.is_shutdown():
            if self.skipScan:
                self.skipScan = False
            else:
                self.scanWorkspace()
            # Get list of objects and classify their ownership
            msg = rospy.wait_for_message("object_db", RichObjectArray)
            objects = self.classify(msg.objects)
            for obj in objects:
                if (not self.inHomeArea(obj) and
                    obj.forbiddenness < self.threshold):
                    ret = self.collect(obj)
                    if ret.response == DoAction.ACT_FAILED:
                        # Update ownership rules and reclassify
                        self.blacklist(obj)
                        self.skipScan = True
                        break

if __name__ == '__main__':
    rospy.init_node('object_collector')
    objectCollector = ObjectCollector()
    objectCollector.main()
    rospy.spin()

