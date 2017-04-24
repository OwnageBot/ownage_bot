#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
import sys

IS_SIMULATION = rospy.get_param("is_simulation")


class ObjectClassifier:
    """A class for classifying objects into ownership categories."""

    def __init__(self):
        self.object_db = dict()
        self.interaction_log = []
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids") if
                             rospy.has_param("landmark_ids") else [])
        rospy.Service("classifyObjects", ClassifyObjects, self.handleClassify)

    def handleClassify(self, req):
        """Classifies each object and returns the list with updated ownership."""
        msg = rospy.wait_for_message("object_db", RichObjectArray)
        objects = msg.objects
        for obj in objects:
            self.classifyObject(obj)
        return ClassifyObjectsResponse(objects)

    def feedbackCallback(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        self.interaction_log.append(msg)

    def classifyObject(self, obj):
        """Modifies the owners and ownership probabilities in place."""
        pass


if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Are we running a simulation?
    if(!IS_SIMULATION):
        # Subscribe to feedback from ObjectCollector
        rospy.Subscriber("feedback", RichFeedback,
                         objectClassifier.feedbackCallback)
    # if so, read fake data from file and do computations

    else:
        data_file = sys.argv[1]
        with open(data_file) as f:
            pass

    rospy.spin()
