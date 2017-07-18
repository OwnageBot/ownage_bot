#!/usr/bin/env python
import rospy
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import Objects

import math
import numpy as np

import copy
import os.path
import csv

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
        self.classify_srv = rospy.Service("classify_objects", ListObjects,
                                          self.handleClassify)
        self.reset_srv = rospy.Service("reset_classifier", std_srvs.srv.Empty,
                                       self.handleReset)
        self.feedback_sub = rospy.Subscriber("feedback", FeedbackMsg,
                                             self.feedbackCb)
                         
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
        return ListObjectsResponse([o.asMessage() for o in objects])

    def handleReset(self, req):
        """Handles reset service request. Clears interaction log."""
        self.object_db = []
        self.interaction_log = []
        self.nb_dist = []
        return std_srs.srv.EmptyResponse()
    
    def feedbackCb(self, msg):
        """Callback upon receiving feedback from ObjectCollector."""
        self.insertNewDist(msg)
        self.interaction_log.append(msg)
        
    def classifyObject(self, obj):
        """Classifies objects using probailistic RBF-kernel method.

        Modifies the owners and ownership probabilities in place."""

        rospy.logdebug("Received obj {} with owners {}".
                       format(obj.id,obj.ownership.iterkeys()))
        
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
            self.ownership[0] = 1.0
            self.object_db[obj.id] = obj
            return
            
        # Normalize weights to get probabilities
        new_probs = {o: w / sum_owner_weights
                     for o, w in owner_weights.items()}

        # Average between old and new ownership
        if obj.id in self.object_db:
            old_probs = dict(self.object_db[obj.id].ownership)
            for o in new_probs:
                if o in old_probs:
                    obj.ownership[o] = (self.learn_rate * new_probs[o] +
                                        (1-self.learn_rate) * old_probs[o])

        self.object_db[obj.id] = obj

        # Print ownership and other properties for debugging
        rospy.logdebug("Object id: {}".format(obj.id))
        rospy.logdebug("Ownership probabilities")
        for k, v in sorted(obj.ownership.iteritems()):
           rospy.logdebug("{}: {}".format(k, v))
        rospy.logdebug("Proximities: {}".format(obj.proximities))
        rospy.logdebug("Color: {}".format(obj.color))

    def dist(self, o1, o2):
        """Computes raw displacement vector between the two objects"""
        
        col_dist = 1.0 if o1.color != o2.color else 0.0

        pos_dist = Objects.dist(o1, o2)
        
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
            for m in range(self.n_dims):
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
        return gradient
        
    def updateMetricWeights(self):
        """Update metric weights through Neighborhood Component Analysis and
        conjugate gradient ascent with backtracking line search."""

        # Objective function is expected number of correct classifications
        objective_f = lambda M : sum(self.computeProbCorrect(M)[1])
        mat_norm = lambda M : sum(sum(np.multiply(M, M)))
        
        step_size = 1.0
        shrinkage = 0.8
        allowance = 0.5

        # No point if less than 3 interactions
        n_interactions = len(self.interaction_log)
        if n_interactions < 3:
            return
        accuracies = [objective_f(None) / n_interactions]
        
        # Conjugate gradient ascent
        cur_grad = self.computeMetricGradient(*self.computeProbCorrect())
        direction = cur_grad
        
        for i in range(self.n_dims):            
            cur_f = objective_f(None)
            grad_norm = mat_norm(cur_grad)

            # Backtracking line search
            while True:
                w_matrix = self.w_matrix + step_size * direction
                new_f = objective_f(w_matrix)
                tgt_f =  cur_f + allowance * step_size * grad_norm
                tgt_f = min(tgt_f, n_interactions)
                if new_f >= tgt_f:
                    break
                step_size *= shrinkage
                
            # Assign new weights
            self.w_matrix = w_matrix
            accuracies.append(new_f / n_interactions)

            # Compute conjugate direction
            cur_grad = self.computeMetricGradient(*self.computeProbCorrect())
            if grad_norm == 0:
                break
            beta = mat_norm(cur_grad) / grad_norm
            direction = cur_grad + beta * direction

        rospy.logdebug("Accuracy: Start - {}, End: {}".
                       format(accuracies[0], accuracies[-1]))

        # Log data if flag is set
        if rospy.get_param("~metric_learn_log", False):
            csv_path = os.path.expanduser("~/metric_learn_log.csv")
            with open(csv_path, 'ab') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["accuracy", "n_interactions", n_interactions])
                writer.writerows([[a] for a in accuracies])
            
if __name__ == '__main__':
    rospy.init_node('object_classifier')
    objectClassifier = ObjectClassifier()
    rospy.spin()