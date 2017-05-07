#!/usr/bin/env python
import rospy
import math
import copy
import numpy as np
import std_msgs.msg
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *

class ObjectClassifier:
    """A class for classifying objects into ownership categories."""

    def __init__(self):
        self.object_db = dict()
        self.interaction_log = []
        self.avatar_ids = []
        self.landmark_ids = []
        self.n_dims = 4
        self.w_color = 30
        self.w_pos = 10
        self.w_prox = 10
        self.learn_rate = 0.5
        self.w_matrix = np.diag([self.w_color] + [self.w_pos] * 3)

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

        # Extract list of avatars and assign proximity weights
        # TODO: deal with arbitrary ordering of avatar ids
        self.avatar_ids = [o.id for o in objects if o.is_avatar]
        self.w_proxs = [self.w_prox] * len(self.avatar_ids)

        # Extend weight matrix to account for new avatars
        new_dims = 4 + len(self.avatar_ids)
        new_matrix = np.diag([self.w_prox] * new_dims)
        new_matrix[:self.n_dims,:self.n_dims] = self.w_matrix
        self.w_matrix = new_matrix
        self.n_dims = new_dims
        
        # Assign new ownership probabilities in place
        if self.interaction_log:
            for obj in objects:
                if not obj.is_avatar:
                    self.classifyObject(obj)
        return ListObjectsResponse(objects)

    def feedbackCallback(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        self.interaction_log.append(msg)

    def resetCallback(self, msg):
        """Callback upon receiving reset switch. Clears interaction log."""
        self.object_db = []
        self.interaction_log = []
        
    def classifyObject(self, obj):
        """Modifies the owners and ownership probabilities in place."""
        rospy.logdebug("Received obj {} with owners {}".
                       format(obj.id,obj.owners))

        def gauss(dist, sigma):
            sqr = lambda x: x*x
            return math.exp(-0.5 * np.dot(dist, dist) / sqr(sigma))

        sigma = 1  # Need to figure out a good value for this
        owner_weights = dict()
        
        # Iterate through interaction history
        for f in self.interaction_log:
            owner = f.label

            # Compute weighted displacement between obj and past object
            dist = self.dist(obj, f.object)
            w_dist = np.dot(self.w_matrix, dist)
            
            # Apply Gaussian weighting to the interaction
            # Compute running sum of weights for each owner id (including 0)
            if owner not in owner_weights.keys():
                owner_weights[owner] = 0
            owner_weights[owner] += gauss(w_dist, sigma)

        # If weights are too small, just assume unowned
        sum_owner_weights = sum(owner_weights.values())
        if sum_owner_weights == 0:
            obj.owners = [0]
            obj.ownership = [1.0]
            self.object_db[obj.id] = obj
            return
            
        # Normalize weights to get probabilities
        owner_probs = {o: w / sum_owner_weights
                       for o, w in owner_weights.items()}

        # Average between old and new ownership
        if obj.id in self.object_db:
            old_owners = self.object_db[obj.id].owners
            old_ownership = self.object_db[obj.id].ownership

            old_probs = dict(zip(old_owners, old_ownership))

            for o in owner_probs:
                if o in old_probs:
                    owner_probs[o] = (self.learn_rate * owner_probs[o] +
                                      (1-self.learn_rate) * old_probs[o])

        obj.owners = owner_probs.keys()
        obj.ownership = owner_probs.values()

        self.object_db[obj.id] = obj

        # Print ownership and other properties for debugging
        rospy.logdebug("Object id: {}".format(obj.id))
        rospy.logdebug("Ownership probabilities")
        for key in sorted(owner_probs):
           rospy.logdebug("{}: {}".format(key, owner_probs[key]))
        rospy.logdebug("Proximities: {}".format(obj.proximities))
        rospy.logdebug("Color: {}".format(obj.color))

    def dist(self, o1, o2):
        """Computes raw displacement vector between the two objects"""
        
        col_dist = 1.0 if o1.color != o2.color else 0.0

        pos = lambda o : o.pose.pose.position
        pos_dist = [pos(o1).x - pos(o2).x,
                    pos(o1).y - pos(o2).y,
                    pos(o1).z - pos(o2).z]
        
        prox_dist = [p1 - p2 for p1, p2 in
                     zip(o1.proximities, o2.proximities)]

        return np.array([col_dist] + pos_dist + prox_dist)
    
if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Subscribe to feedback from ObjectCollector / ObjectTester
    rospy.Subscriber("feedback", RichFeedback,
                     objectClassifier.feedbackCallback)
    rospy.Subscriber("reset_classifier", std_msgs.msg.Empty,
                     objectClassifier.resetCallback)
    rospy.spin()
