#!/usr/bin/env python
import rospy
import math
import copy
import std_msgs.msg
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *

class ObjectClassifier:
    """A class for classifying objects into ownership categories."""

    def __init__(self):
        self.object_db = dict()
        self.interaction_log = []
        self.avatar_ids = (rospy.get_param("avatar_ids")
                           if rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids")
                             if rospy.has_param("landmark_ids") else [])
        self.w_color = 5
        self.w_pos = 1
        self.w_proxs = [1] *  len(self.avatar_ids)

        self.listObjects = rospy.ServiceProxy("list_objects", ListObjects)
        rospy.Service("classify_objects", ListObjects, self.handleClassify)

    def handleClassify(self, req):
        """Classifies and returns a list of objects."""
        rospy.loginfo("Classifying objects...\n")
        resp = self.listObjects()

        # Initialize object database assuming that all objects are unowned
        if len(self.object_db) == 0:
            object_ids = [obj.id for obj in resp.objects]
            self.object_db = dict(zip(object_ids, copy.deepcopy(resp.objects)))

        objects = resp.objects
        # Assign new ownership probabilities in place
        if self.interaction_log:
            for obj in objects:
                if not obj.is_avatar:
                    self.classifyObject(obj)
        return ListObjectsResponse(objects)

    def feedbackCallback(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        # if len(self.interaction_log) == 0:
        #     resp = self.listObjects()
        #     for o in resp.objects:
        #         for a in [0] + self.avatar_ids:
        #             fb = RichFeedback()
        #             fb.object = o
        #             fb.label = a
        #             self.interaction_log.append(fb)
        self.interaction_log.append(msg)

    def resetCallback(self, msg):
        """Callback upon receiving reset switch. Clears interaction log."""
        self.interaction_log = []
        
    def classifyObject(self, obj):
        """Modifies the owners and ownership probabilities in place."""
        obj.owners = list(obj.owners)
        obj.ownership = list(obj.ownership)

        sigma = 1  # Need to figure out a good value for this
        total_weight_sum = 0

        sum_of_weights = dict(zip(obj.owners, [0] * len(obj.owners)))
        rospy.loginfo("Received obj {} with owners {}\n".
            format(obj.id,obj.owners))

        def gauss(dist, sigma):
            sqr = lambda x: x*x
            return math.exp(-.5 * sqr(dist / sigma))

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
                sum_of_weights[label] += gauss(dist, sigma)
            else:
                sum_of_weights[label] = gauss(dist, sigma)

        total_weight_sum = sum(sum_of_weights.values())

        if total_weight_sum:
            for k in sum_of_weights.keys():
                # Compute running sum of weights for each avatar id (including 0)
                # Normalize weights to get probabilities
                obj.ownership[obj.owners.index(k)] = (sum_of_weights[k] /
                                                      total_weight_sum)
        else:
            obj.ownership = [0] * len(sum_of_weights)
            obj.ownership[0] = 1.0

        ownerDict = dict(zip(obj.owners, obj.ownership))

        # Average between old and new ownership
        if obj.id in self.object_db:
            old_owners = self.object_db[obj.id].owners
            old_ownership = self.object_db[obj.id].ownership

            oldOwnerDict = dict(zip(old_owners, old_ownership))

            for k in ownerDict:
                if k in oldOwnerDict:
                    ownerDict[k] = ownerDict[k] + oldOwnerDict[k]
            for k in ownerDict:
                ownerDict[k] = ownerDict[k] / 2.0

            obj.owners = ownerDict.keys()
            obj.ownership = ownerDict.values()

        self.object_db[obj.id] = obj

        print "Object id: %s" % obj.id
        for key in sorted(ownerDict):
           print "%s: %s" % (key, ownerDict[key])
        print(obj.proximities)
        print("Color: {}".format(obj.color))


    def norm(self, o, f):
        """Determines the squared L2 norm between two objects"""
        o_color = o.color
        f_color = f.object.color
        color_dist = 1.0 if o_color != f_color else 0.0

        # weights used to normalize data
        sum_sqr_proxs = 0

        sqr = lambda x: x*x

        for i in range(len(o.proximities)):
            sum_sqr_proxs +=  self.w_proxs[i] * sqr(
                o.proximities[i] - f.object.proximities[i])

        pos = lambda o : o.pose.pose.position

        sum_sqr_pos = (
            sqr(pos(o).x - pos(f.object).x) +
            sqr(pos(o).y - pos(f.object).y) +
            sqr(pos(o).z - pos(f.object).z))

        return (self.w_color * color_dist +
                sum_sqr_proxs +
                self.w_pos * sum_sqr_pos)

if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Subscribe to feedback from ObjectCollector / ObjectTester
    rospy.Subscriber("feedback", RichFeedback,
                     objectClassifier.feedbackCallback)
    rospy.Subscriber("reset_classifier", std_msgs.msg.Empty,
                     objectClassifier.resetCallback)
    rospy.spin()
