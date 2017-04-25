#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
import math
import sys

IS_SIMULATION = rospy.get_param("is_simulation")


class ObjectClassifier:
    """A class for classifying objects into ownership categories."""

    def __init__(self):
        self.object_db = dict()
        self.interaction_log = []
        self.avatar_ids = (rospy.get_param("avatar_ids")
                           if rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids")
                             if rospy.has_param("landmark_ids") else [])
        self.w_color = 1
        self.w_pos = 1
        self.w_proxs = [1]*len(self.landmark_ids)

        rospy.Service("classifyObjects", ClassifyObjects, self.handleClassify)

    def handleClassify(self, req):
        """Classifies each object and returns the list with updated ownership."""
        msg = rospy.wait_for_message("object_db", RichObjectArray)
        objects = msg.objects
        if self.interaction_log:
            for obj in objects:
                self.classifyObject(obj)
        else: return False

        return ClassifyObjectsResponse(objects)

    def feedbackCallback(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        self.interaction_log.append(msg)

    def classifyObject(self, obj):
        """Modifies the owners and ownership probabilities in place."""
        sigma = 1  # Need to figure out a good value for this
        total_weight_sum = 0

        sum_of_weights = {}

        for f in self.interaction_log:
            label = f.label
            if label not in obj.owners:
                obj.owners.append(label)
                obj.ownership.append(0.0)
            # Compute distance between i and obj
            dist = self.norm(obj, f)

            # Apply Gaussian weighting to the interaction
            # Compute running sum of weights for each avatar id (including 0)
            if label in sum_of_weights:
                sum_of_weights[label] += self.gauss(dist, sigma)
            else:
                sum_of_weights[label] = self.gauss(dist, sigma)

        total_weight_sum = sum(sum_of_weights.values())

        for k in sum_of_weights.keys():
            # Compute running sum of weights for each avatar id (including 0)
            # Normalize weights to get probabilities
            obj.ownership[obj.ownership.index(k)] = (sum_of_weights[k] /
                                                     total_weight_sum)

    def norm(o, f):
        """Determines the squared L2 norm of two objects"""
        o_color = o.color
        f_color = f.object.color
        color_dist = 1.0 if o_color != f_color else 0.0

        # weights used to normalize data
        sum_sqr_proxs = 0

        sqr = lambda x: x*x

        for i in range(len(o.proximities)):
            sum_sqr_proxs +=  self.w_proxs[i] * sqr(
                o.proximities[i] - f.object.proximities[i])


        sum_sqr_pos = (
            sqr(o.pose.pose.x - f.object.pose.pose.x) +
            sqr(o.pose.pose.y - f.object.pose.pose.y) +
            sqr(o.pose.pose.z - f.object.pose.pose.z))

        return (self.w_color * color_dist +
                sum_sqr_proxs +
                self.w_pos * sum_sqr_pos)

    def gauss(dist, sigma):
        sqr = lambda x: x*x
        return math.exp(-.5 * sqr(dist / sigma))


if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Are we running a simulation?
    if not IS_SIMULATION:
        # Subscribe to feedback from ObjectCollector
        rospy.Subscriber("feedback", RichFeedback,
                         objectClassifier.feedbackCallback)
    # if so, read fake data from file and do computations

    else:
        data_file = sys.argv[1]
        with open(data_file) as f:
            pass

    rospy.spin()
