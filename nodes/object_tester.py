#!/usr/bin/env python
import rospy
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import Objects

import math
import random as r
import os.path
import csv

OBJECT_BASE_ID = 1
AVATAR_BASE_ID = 100

class ObjectTester():
    """Generates a simulated environment of objects and avatars
    to test and train the learning algorithm in ObjectClassifier.
    """
    
    def __init__(self, metrics, n_objs, n_avatars, n_examples, threshold=0.2):
        """Constructs a tester with a corresponding test environment.

        metrics -- Cluster based on either 'color', 'position', 'proximity',
                   or some combination of them.
        n_objs -- Number of objects in the environment (excluding avatars)
        n_avatars -- Number of avatars
        n_examples -- Number of training examples, drawn from n_objs
        threshold -- Ownership cutoff for object collection (online training)

        """

        if not set(metrics).issubset(['color', 'position', 'proximity']):
            rospy.logerr("One or more metrics are unrecognized!")
            rospy.logerr("Allowed metrics: 'color', 'position', 'proximity.")
            return        
        if n_examples > n_objs:
            rospy.logerr("Cannot have more examples than objects!")
            return
            
        self.metrics = metrics
        self.n_avatars = n_avatars
        self.n_objs = n_objs
        self.n_examples = n_examples
        self.threshold = threshold

        # If true, will collect most likely unowned objects
        self.greedy = rospy.get_param("~greedy", True)
        
        self.avatars = []
        self.objects = [] # Non-avatar objects
        self.labels = [] # True owner labels for non-avatar objects
        self.centers = [] # Cluster centers for position-based clustering
        self.home = geometry_msgs.msg.Point() # Home location
        
        # Set up service calls to ObjectClassifier as in-class method
        self.classify = rospy.ServiceProxy("classify_objects", ListObjects)
        self.reset = rospy.ServiceProxy("reset_classifier", std_srvs.srv.Empty)

        # Create publishers and servers
        self.feedback_pub = rospy.Publisher("feedback",
                                            RichFeedback,
                                            queue_size=10)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.handleList)

    def handleList(self, req):
        """ Returns list of all tracked objects (including avatars)"""
        return ListObjectsResponse([o.asMessage for o in
                                    (self.avatars + self.objects)])

    def generateEnvironment(self):
        """Randomly generates simulated environment."""
        # Clear previous generated data
        self.avatars = []
        self.centers = []
        self.objects = []
        self.labels = []

        # Set home location
        self.home.x = 0.0
        self.home.y = 0.0
        self.home.z = 0.0

        self.generateAvatars()
        self.generateCenters()
        self.generateObjects()
        
    def generateAvatars(self):
        """Randomly generates the locations of avatars.

        Ensures they are sufficiently spaced apart by generating them 0.4-0.8
        meters from the home location, each within its own angular sector.
        """
        for i in range(self.n_avatars):
            av = RichObject()
            av.id = AVATAR_BASE_ID + i
            av.is_avatar = True

            dist = r.uniform(0.4, 0.8)
            angle = r.uniform(i, i+1) * 2 * math.pi / self.n_avatars
            
            av.position.x = dist * math.cos(angle)
            av.position.y = dist * math.sin(angle)
            av.position.z = 0.0

            self.avatars.append(av)

    def generateCenters(self):
        """Randomly generates the centers of physical clusters.

        If clustering by absolute position, ensures they are sufficiently 
        spaced apart by generating them 0.4-0.8 meters from the origin, each
        within its own angular sector.

        """
        if 'proximity' in self.metrics:
            # Centers are avatar locations if clustering by proximity
            self.centers = ([self.home] +
                            [av.pose.pose.position for av in self.avatars])
        else:
            # Otherwise, just cluster based on absolute position
            for i in range(self.n_avatars + 1):
                c = geometry_msgs.msg.Point()

                dist = r.uniform(0.4, 0.8)
                angle = r.uniform(i, i+1) * 2 * math.pi / (self.n_avatars + 1)
                
                c.x = dist * math.cos(angle)
                c.y = dist * math.sin(angle)
                c.z = 0.0
                self.centers.append(c)

    def generateObjects(self):
        """Randomly generates objects within their clusters.

        Ensures they are tightly clustered (within 0.3m) around centers if
        clustering by position or proximity. Otherwise, they are distributed
        uniformly at random across the workspace.
        """
        oids = range(OBJECT_BASE_ID, OBJECT_BASE_ID + self.n_objs)
        for i, oid in enumerate(oids):
            obj = Objects.Object()
            obj.id = oid

            # Distribute objects equally across clusters
            cluster = i % (self.n_avatars + 1)
            label = 0 if cluster == 0 else self.avatars[cluster-1].id

            # Generate object locations
            if ('position' in self.metrics or 'proximity' in self.metrics):
                # Cluster around centers
                r_offset = r.uniform(0, 0.3)
                r_angle = r.uniform(0, 1) * 2 * math.pi
                c = self.centers[cluster]

                obj.position.x = c.x + r_offset * math.cos(r_angle)
                obj.position.y = c.y + r_offset * math.sin(r_angle)
                obj.position.z = c.z
            else:
                # Uniform distribution if no position or proximity clustering
                obj.position.x = r.uniform(-1.0, 1.0)
                obj.position.y = r.uniform(-1.0, 1.0)
                obj.position.z = r.uniform(-1.0, 1.0)                
                
            # Generate object colors
            if 'color' in self.metrics:
                # Associate cluster with color 
                obj.color = cluster
            else:
                # Choose color uniformly at random
                obj.color = r.choice(range(len(self.avatars) + 1))
                            
            obj.is_avatar = False
            obj.proximities = [Objects.dist(av, obj) for av in self.avatars]
            obj.ownership[0] = 1.0

            self.objects.append(obj)
            self.labels.append(label)
        
    def trainOffline(self):
        """Provides classifier with all training examples before testing."""
        # Select random subset of size n_examples to train classifier
        self.classify() # Ensure classifier knows the avatars
        for obj, label in r.sample(zip(self.objects, self.labels),
                                   self.n_examples):
            fb = RichFeedback()
            fb.object = obj
            fb.label = label
            rospy.loginfo("Chose object {} with owner {} as example".
                          format(obj.id, label))
            rospy.sleep(0.01)
            self.feedback_pub.publish(fb)
        
        resp = self.classify()
        return self.evaluate(resp.objects)

    def trainOnline(self):
        """Reclassifies after each example is given.

        Replicates the behavior of ObjectCollector by picking the object that
        is most likely to be unowned each time, stopping if the probability of
        non-ownership (for all objects) does not exceed the threshold.

        Evaluates and returns three performance metrics:

        f_collected -- The fraction of unowned objects which are collected
                       within the alloted number of interactions (n_examples)
        efficiency -- Ratio of number of trials required to successfully
                      collect all objects to the total number of objects
        accuracy -- The prediction accuracy of the classifier after having
                    been trained on all examples

        """
        collected_ids = []
        n_unowned = self.labels.count(0)
        n_tries = -1
        accuracy = 0

        # Ensure classifier sees the environment / knows the avatars
        self.classify()
        # Randomly pick first object
        obj, label = r.choice(zip(self.objects, self.labels))
        
        for i in range(self.n_examples):
            # Simulated collection (only if object is unowned)
            if label == 0 or not self.greedy:
                collected_ids.append(obj.id)
            if n_tries == -1 and len(collected_ids) == n_unowned:
                n_tries = i+1
                
            # Simulated feedback
            fb = RichFeedback()
            fb.object = obj
            fb.label = label
            rospy.loginfo("Chose object {} with owner {} as example".
                          format(obj.id, label))
            rospy.sleep(0.01)
            self.feedback_pub.publish(fb)
            
            # Classify and filter out avatars
            resp = self.classify()
            objects = [o for o in resp.objects if not o.is_avatar]

            # Check if any more objects to collect
            uncollected = [o for o in objects if o.id not in collected_ids]
            if len(uncollected) == 0:
                rospy.loginfo("All objects have been collected.")
                break

            # If greedy, choose uncollected and most likely unowned object
            obj = (max(uncollected, key=lambda o: o.ownership[0])
                   if self.greedy else r.choice(uncollected))
            label = self.labels[objects.index(obj)]

            # Stop training if all objects are too likely to be owned
            if obj.ownership[0] < self.threshold:
                rospy.loginfo("All objects too likely to be owned.")
                rospy.loginfo("Stopping after {} examples.".format(i))
                break

        # Compute performance metrics
        f_collected = (float(len(collected_ids)) / n_unowned
                       if n_unowned > 0 else -1)
        efficiency = (float(n_tries) / self.n_objs
                      if n_tries > 0 else -1)
        accuracy = self.evaluate(objects)
        
        print("Collected fraction: {} ({}/{})".
              format(f_collected, len(collected_ids), n_unowned))
        print("Collection efficiency: {} ({}/{})".
              format(efficiency, n_tries, self.n_objs))
        print("Final classification accuracy: {}".
              format(accuracy))

        return (f_collected, efficiency, accuracy)
        
    def evaluate(self, classified):
        """Evaluates and returns classifier accuracy."""
        predictions = []
        for o in classified:
            if o.is_avatar:
                continue
            # Prediction is the owner with maximum ownership probability
            predicted = max(o.ownership, key=lambda k:o.ownership[k])
            predictions.append(predicted)

        # Assumes that predicted labels are in the same order as true labels
        correct = [(actual == predicted) for (actual, predicted) in
                   zip(self.labels, predictions)]
        n_correct = sum(correct)

        accuracy = float(n_correct) / self.n_objs
        print("Accuracy: {} ({}/{})".format(accuracy, n_correct, self.n_objs))

        return accuracy

if __name__ == '__main__':
    rospy.init_node('object_tester')

    # Load parameters from launch file
    online = rospy.get_param("~online", True)
    trials = rospy.get_param("~trials", 100)
    n_objs = rospy.get_param("~n_objs", 10)
    n_avatars = rospy.get_param("~n_avatars", 2)
    n_examples = rospy.get_param("~n_examples", 3)
    metrics = rospy.get_param("~metrics", ['color', 'position', 'proximity'])
    threshold = rospy.get_param("collect_threshold", 0.2)

    tester = ObjectTester(metrics, n_objs, n_avatars, n_examples, threshold)
    results = []

    # Run tester for multiple trials
    for t in range(trials):
        tester.generateEnvironment()
        rospy.wait_for_service('classify_objects')
        if online:
            results.append(tester.trainOnline())
        else:
            results.append([tester.trainOffline()])
        tester.reset()
        rospy.sleep(0.01)

    # Average and print results
    print("")
    print("AGGREGATE RESULTS")
    if online:
        f_collected, efficiency, accuracy = zip(*results)
        
        f_collected = [r for r in f_collected if r >= 0]
        efficiency = [r for r in efficiency if r >= 0]
        accuracy = [r for r in accuracy if r >= 0]
        
        avg_f_collected = (sum(f_collected) / len(f_collected)
                           if len(f_collected) > 0 else -1)
        avg_efficiency = (sum(efficiency) / len(efficiency)
                          if len(efficiency) > 0 else -1)
        avg_accuracy = sum(accuracy) / len(accuracy)

        print("Overall fraction collected: {}".
              format(avg_f_collected))
        print("Overall collection efficiency: {}".
              format(avg_efficiency))
        print("Overall classification accuracy: {}".
              format(avg_accuracy))
    else:
        avg_accuracy = sum(results[0]) / len(results[0])
        print("Simulated training for {} trials complete.".format(trials))
        print("Overall accuracy: {}".format(avg_accuracy))
        
    # Save results to CSV file
    csv_path = os.path.expanduser("~/ownage_bot_performance.csv")
    with open(csv_path, 'wb') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["metrics:"] + metrics)
        writer.writerow(["n_objs:", n_objs,
                         "n_avatars:", n_avatars,
                         "n_examples:", n_examples,
                         "threshold:", threshold])
        if online:
            writer.writerow([avg_f_collected, avg_efficiency, avg_accuracy])
            writer.writerow(["f_collected", "efficiency", "accuracy"])
        else:
            writer.writerow([avg_accuracy])            
            writer.writerow(["accuracy"])            
        writer.writerows(results)
