#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from baxter_collaboration_msgs.srv import DoAction
from ownage_bot.msg import RichObject
from ownage_bot.msg import RichObjectArray
from geometry_msgs.msg import Point

DUMMY_OBJ = -1

class ObjectCollector:
    """A class for collecting objects."""

    def __init__(self):
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

    def objectDbCallback(self):
        """Callback upon receiving updated database from ObjectTracker."""
        # Interrupt search and pick up new objects
        pass

    def scanWorkspace(self):
        return self.actionProvider("scan", [DUMMY_OBJ])

    def goHome(self):
        return self.actionProvider("home", [DUMMY_OBJ])

    def pickUp(self, obj):
        return self.actionProvider("get", [obj.id])

    def putDown(self):
        return self.actionProvider("put", [DUMMY_OBJ])

    def collect(self, obj):
        resp = self.pickUp(obj)
        if not resp.success:
            return resp
        resp = self.goHome()
        if not resp.success:
            return resp
        resp = self.putDown()
        if not resp.success:
            return resp        
    
    def inHomeArea(self, obj):
        loc = obj.pose.pose.position
        return ((self.home_area[0].x <= loc.x <= self.home_area[1].x) and
                (self.home_area[0].y <= loc.y <= self.home_area[1].y))
        
    def main(self):
        """Main loop which collects all tracked objects."""
        while not rospy.is_shutdown():
            self.scanWorkspace()
            msg = rospy.wait_for_message("object_db", RichObjectArray)
            for obj in msg.objects:
                if (not self.inHomeArea(obj) and
                    obj.forbiddenness < self.threshold):
                    self.collect(obj)                    

if __name__ == '__main__':
    rospy.init_node('object_collector')
    objectCollector = ObjectCollector()
    objectCollector.main()
    rospy.spin()

