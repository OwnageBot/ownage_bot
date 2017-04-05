#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
import geometry_msgs.msg

class ObjectCollector:
    """A class for tracking objects."""

    def __init__(self, latency = 0.2):
        self.latency = latency # In seconds
   
    def collect(self):
    """Main loop which collects all tracked objects."""
        r = rospy.Rate(1/self.latency)
        while not rospy.is_shutdown():
            # Check if all tracked objects are in home area
            # Collect tracked objects which are not in home area
            # Search for new objects
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('object_collector')
    objectCollector = ObjectCollector()
    objectCollector.collect()
    rospy.spin()

