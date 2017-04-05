#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from baxter_collaboration_msgs.srv import DoAction
from ownage_bot.msg import RichObject
from ownage_bot.msg import RichObjectArray
import geometry_msgs.msg

class ObjectCollector:
    """A class for collecting objects."""

    def __init__(self, latency = 0.2):
        self.latency = latency # In seconds
        self.is_searching = 1
        self.actionProvider = rospy.ServiceProxy(
            "/action_provider/service_left", DoAction)

    def objectDbCallback(self):
        """Callback upon receiving updated database from ObjectTracker."""
        # Interrupt search and pick up new objects
        pass

    def scanWorkspace(self):
        # TODO: call actionProvider with relevant input
    
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
    # Subcribe to database updates from object_tracker
    rospy.Subscriber("/object_tracker/object_db",
                     RichObjectArray, objectCollector.objectDbCallback)
    objectCollector.collect()
    rospy.spin()

