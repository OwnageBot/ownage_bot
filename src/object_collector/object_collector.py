#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from baxter_collaboration_msgs.srv import DoAction
from ownage_bot.msg import RichObject
from ownage_bot.msg import RichObjectArray
from geometry_msgs.msg import Point

class ObjectCollector:
    """A class for collecting objects."""

    def __init__(self):
        # Threshold for forbiddenness
        self.threshold = 0.8
        # Rectangle denoting home area
        self.home_area = (Point(0.45,-0.2, 0), Point(0.55, 0.2, 0))
        self.actionProvider = rospy.ServiceProxy(
            "/action_provider/service_left", DoAction)

    def objectDbCallback(self):
        """Callback upon receiving updated database from ObjectTracker."""
        # Interrupt search and pick up new objects
        pass

    def scanWorkspace(self):
        return self.actionProvider("scan", [])

    def goHome(self):
        return self.actionProvider("home", [])

    def pickUp(self, obj):
        return self.actionProvider("get", [obj.id])

    def putDown(self):
        return self.actionProvider("put", [])

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
            msg = rospy.wait_for_message(
                "/object_tracker/object_db", RichObjectArray)
            for obj in msg.objects:
                if (not self.inHomeArea(obj) and
                    obj.forbiddenness < self.threshold):
                    self.collect(obj)                    

if __name__ == '__main__':
    rospy.init_node('object_collector')
    objectCollector = ObjectCollector()
    objectCollector.main()
    rospy.spin()

