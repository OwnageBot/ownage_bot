#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
import geometry_msgs.msg
from ownage_bot.msg import RichObject
from ownage_bot.srv import ClassifyObjects

class ObjectClassifier:
    """A class for classifying objects into ownership categories."""

    def __init__(self):
        self.object_db = dict()
        self.color_db = {} # Which colors have we seen?
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids") if
                             rospy.has_param("landmark_ids") else [])
        rospy.Service("classifyObjects", ClassifyObjects, self.handleClassify)

    def handleClassify(self, req):
        objects = req.unclassified
        return ClassifyObjectsResponse(objects)
        
    def blacklistCallback(self, msg):
        """Callback upon receiving blacklisted object."""
        pass

if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Published by aruco_ros
    rospy.Subscriber("blacklist", RichObject,
                     objectClassifier.blacklistCallback)
    rospy.spin()

