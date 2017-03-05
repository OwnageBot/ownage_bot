#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
import geometry_msgs.msg

class RichObject:
    """Objects with additional properties."""
    def __init__(self):
        self.ARuco_id = -1;
        self.local_pose = geometry_msgs.msg.PoseWithCovariance()
        self.color = (0,0,0)
        self.last_movement = 0
        self.forbiddenness = 0
        self.is_avatar = 0
        self.is_landmark = 0

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self):
        self.object_db = []
        self.avatar_ids = []
        self.landmarks_ids = []
        self.new_obj_pub = rospy.Publisher("new_object", UInt32, queue_size = 10)
   
    def ARucoCallback(self, msg):
        for m in msg.markers:
            # Check if object is already in database
            if len(filter(lambda obj: obj.id == m.id, self.object_db)) == 0:
                obj = RichObject()
                obj.ARuco_id = m.id
                obj.local_pose = m.pose # Actually PoseWithCovariance
                obj.color = self.determineColor(m.pose)
                obj.is_avatar = (m.id in self.avatar_ids)
                obj.is_landmark = (m.id in self.landmark_ids)
                self.object_db.append(obj)
                self.new_obj_pub(m.id) # Publish that new object was found
     
     def determineColor(self, pose):
        return (0, 0, 0)
     
if __name__ == '__main__':
    rospy.init_node('object_tracker', anonymous=True)
    objectTracker = ObjectTracker()
    rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, objectTracker.ARucoCallback)
    rospy.spin()

