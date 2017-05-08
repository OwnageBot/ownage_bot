#!/usr/bin/env python
import rospy
import math
import copy
import numpy as np
import itertools as itools
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

        self.learn_rate = 0.5

        self.n_dims = 4
        self.w_color = rospy.get_param("~w_color", 30)
        self.w_pos = rospy.get_param("~w_pos", 10)
        self.w_prox = rospy.get_param("~w_prox", 10)
        self.w_matrix = np.diag([self.w_color] + [self.w_pos] * 3)
        
        self.nb_dist = []
        
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
        pad_width = new_dims - self.n_dims
        self.w_matrix = np.pad(self.w_matrix, (0,pad_width), 'constant')
        for i in range(self.n_dims, new_dims):
            self.w_matrix[i,i] = self.w_prox
        self.n_dims = new_dims

        # Adjust weights if necessary
        self.updateMetricWeights()
        
        # Assign new ownership probabilities in place
        if self.interaction_log:
            for obj in objects:
                if not obj.is_avatar:
                    self.classifyObject(obj)
        return ListObjectsResponse(objects)
    
    def feedbackCallback(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        self.insertNewDist(msg)
        self.interaction_log.append(msg)

    def resetCallback(self, msg):
        """Callback upon receiving reset switch. Clears interaction log."""
        self.object_db = []
        self.interaction_log = []
        self.nb_dist = []
        
    def classifyObject(self, obj):
        """Classifies objects using probailistic RBF-kernel method.

        Modifies the owners and ownership probabilities in place."""

        rospy.logdebug("Received obj {} with owners {}".
                       format(obj.id,obj.owners))

        gauss = lambda x : math.exp(-0.5 * np.dot(x, x))
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
            owner_weights[owner] += gauss(w_dist)

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
    
    def insertNewDist(self, new):
        """Compute distance to previous interactions and insert into table."""
        i = len(self.interaction_log)
        self.nb_dist.append([np.zeros(self.n_dims)] * (i+1))
        for j, old in enumerate(self.interaction_log):
            dist = self.dist(new.object, old.object)
            self.nb_dist[i][j] = dist
            self.nb_dist[j].append(dist)

    def computeProbCorrect(self, w_matrix=None):
        """Compute probabilities of correctness upon self-classification i.e.,
        if each object in the interaction history were classified against 
        the others.

        w_matrix -- Optionally use some other weights for the metric

        Returns (nb_probs, owner_probs, true_neighbors):

        nb_probs[i,j] -- Probability that i, j are neighbors (i.e same owner)
        owner_probs[i] -- Probability that i is correctly classified
        true_neighbors[i] -- List of actual neighbors of the ith interaction
        """

        if w_matrix is None:
            w_matrix = self.w_matrix
        
        n_interactions = len(self.interaction_log)
        nb_probs = np.empty((n_interactions, n_interactions))
        owner_probs = [0] * n_interactions
        true_neighbors = [list() for i in range(n_interactions)]

        gauss = lambda x : math.exp(-0.5 * np.dot(x, x))
        sim = lambda d : gauss(np.dot(w_matrix, d))
        
        for i, fb1 in enumerate(self.interaction_log):
            dists = self.nb_dist[i]
            similarities = map(sim, dists)
            sum_sim = sum(similarities) - similarities[i]
            for j, fb2 in enumerate(self.interaction_log):
                # Compute probability that i and j have the same owner
                nb_probs[i,j] = (similarities[j] / sum_sim
                                 if (i != j and sum_sim > 0) else 0.0) 
                # Update probability that i is correctly classified
                if fb1.label == fb2.label:
                    owner_probs[i] += nb_probs[i,j]
                    true_neighbors[i].append(j)

        return (nb_probs, owner_probs, true_neighbors)

    def computeMetricGradient(self, nb_probs, owner_probs, true_neighbors):
        """Compute gradient of objective with respect to metric weights,
        where objective is the expected number of correctly classified points.

        Refer to NCA paper by Goldberger et al. for the symbolic expression.
        """
        gradient = np.zeros((self.n_dims, self.n_dims))
        n_interactions = len(self.interaction_log)
        # Iterate over each pair of metric dimensions (n,m)
        for n in range(self.n_dims):
            for m in range(n, self.n_dims):
                dpdw = 0 # Running sum
                for i in range(n_interactions):
                    # Some kind of probabilistic covariance measure
                    prob_dim_cov = [nb_probs[i,j] *
                                    self.nb_dist[i][j][n] *
                                    self.nb_dist[i][j][m] for
                                    j in range(n_interactions)]
                    dpdw += (owner_probs[i] * sum(prob_dim_cov) -
                             sum([prob_dim_cov[j] for j in true_neighbors[i]]))
                dpdw *= 2 * self.w_matrix[n,m]
                gradient[n,m] = dpdw
                gradient[m,n] = dpdw
        return gradient
        
    def updateMetricWeights(self):
        """Update metric weights through Neighborhood Component Analysis and
        one iteration of gradient ascent with backtracking line search."""

        # Objective function is expected number of correct classifications
        objective_f = lambda M : sum(self.computeProbCorrect(M)[1])
        step_size = 1.0
        shrinkage = 0.8
        allowance = 0.5

        n_interactions = len(self.interaction_log)
        
        for i in range(1):   
            gradient = self.computeMetricGradient(*self.computeProbCorrect())
            cur_f = objective_f(None)
            
            # Gradient ascent with backtracking line search
            while True:
                w_matrix = self.w_matrix + step_size * gradient
                grad_norm = sum(sum(np.multiply(gradient, gradient)))
                new_f = objective_f(w_matrix)
                tgt_f =  cur_f + allowance * step_size * grad_norm
                tgt_f = min(tgt_f, n_interactions)
                if new_f >= tgt_f:
                    break
                step_size *= shrinkage

            if self.interaction_log:
                acc = [f/n_interactions for f in [cur_f, new_f, tgt_f]]
                print("Cur: {}, New: {}, Tgt: {}".format(*acc))
                
            # Assign new weights
            self.w_matrix = w_matrix
            print w_matrix
                    
if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    # Subscribe to feedback from ObjectCollector / ObjectTester
    rospy.Subscriber("feedback", RichFeedback,
                     objectClassifier.feedbackCallback)
    rospy.Subscriber("reset_classifier", std_msgs.msg.Empty,
                     objectClassifier.resetCallback)
    rospy.spin()
