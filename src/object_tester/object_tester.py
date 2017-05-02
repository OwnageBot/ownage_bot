#!/usr/bin/env python
import rospy
import std_msgs.msg
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
import math
import random as r

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

        if not set(['color', 'position', 'proximity']).issubset(metrics):
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
        
        self.avatars = []
        self.objects = [] # Non-avatar objects
        self.labels = [] # True owner labels for non-avatar objects
        self.centers = [] # Cluster centers for position-based clustering
        self.home = geometry_msgs.msg.Point() # Home location
        
        # Set up service call to ObjectClassifier as in-class method
        self.classify = rospy.ServiceProxy("classifyObjects", ListObjects)

        # Create publishers and servers
        self.feedback_pub = rospy.Publisher("feedback",
                                            RichFeedback,
                                            queue_size=10)
        self.reset_pub = rospy.Publisher("reset_classifier",
                                         std_msgs.msg.Empty,
                                         queue_size=10)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.handleList)

    def handleList(self, req):
        """ Returns list of all tracked objects (including avatars)"""
        return ListObjectsResponse(self.avatars + self.objects)

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
        """Randomly generates the locations of avatars."""
        for i in range(self.n_avatars):
            av = RichObject()
            av.id = AVATAR_BASE_ID + i
            av.is_avatar = True

            av.pose.pose.position.x = r.uniform(-1.0, 1.0)
            av.pose.pose.position.y = r.uniform(-1.0, 1.0)
            av.pose.pose.position.z = 0.0

            self.avatars.append(av)

    def generateCenters(self):
        """Randomly generates the centers of physical clusters."""
        for p in ([self.home] +
                  [av.pose.pose.position for av in self.avatars]):
            if 'proximity' in self.metrics:
                # Centers are avatar locations if clustering by proximity
                self.centers.append(p)
            else:
                # Otherwise, just cluster based on absolute position
                c = geometry_msgs.msg.Point()
                c.x = r.uniform(-1.0, 1.0)
                c.y = r.uniform(-1.0, 1.0)
                c.z = 0.0
                self.centers.append(c)

    def generateObjects(self):
        """Randomly generates objects within their clusters."""
        oids = range(OBJECT_BASE_ID, OBJECT_BASE_ID + self.n_objs)
        for i, oid in enumerate(oids):
            obj = RichObject()
            obj.id = oid

            # Choose cluster / owner label uniformly at random
            cluster = r.choice(range(len(avatars) + 1))
            label = 0 if cluster == 0 else avatars[cluster-1].id

            # Generate object locations
            if ('position' in self.metrics or 'proximity' in self.metrics):
                # Cluster around centers
                rand_offset = r.uniform(-0.5, 0.5)
                c = centers[cluster]

                obj.pose.pose.position.x = c.x + rand_offset
                obj.pose.pose.position.y = c.y + rand_offset
                obj.pose.pose.position.z = c.z
            else:
                # Uniform distribution if no position or proximity clustering
                obj.pose.pose.position.x = r.uniform(-1.0, 1.0)
                obj.pose.pose.position.y = r.uniform(-1.0, 1.0)
                obj.pose.pose.position.z = r.uniform(-1.0, 1.0)                
                
            # Generate object colors
            if 'color' in self.metrics:
                # Associate cluster with color 
                obj.color = cluster
            else:
                # Choose color uniformly at random
                obj.color = r.choice(range(len(avatars) + 1))
                
            # Fill in other object properties

            def dist(obj1, obj2):
                p1 = obj1.pose.pose.position
                p2 = obj2.pose.pose.position
                return math.sqrt((p1.x-p2.x)*(p1.x-p2.x) +
                                 (p1.y-p2.y)*(p1.y-p2.y))
            
            obj.is_avatar = False
            obj.proximities = [dist(av, obj) for av in avatars]
            obj.owners = [0]
            obj.ownership = [1.0]

            self.objects.append(obj)
            self.labels.append(label)
        
    def trainOffline(self):
        """Provides classifier with all training examples before testing."""
        # Select random subset of size n_examples to train classifier
        for obj, label in r.sample(zip(self.objects, self.labels),
                                   self.n_examples):
            fb = RichFeedback()
            fb.object = obj
            fb.label = label
            rospy.loginfo("Chose object {} with owner {} as example".
                          format(obj.id, label))
            rospy.sleep(0.1)
            self.feedback_pub.publish(fb)
        
        resp = self.classify()
        return self.evaluate(resp.objects)

    def trainOnline(self):
        """Reclassifies after each example is given.

        Replicates the behavior of ObjectCollector by picking the object that
        is most likely to be unowned each time, stopping if the probability of
        non-ownership (for all objects) does not exceed the threshold.

        Returns the collection accuracy, average classification accuracy, and
        final classification accuracy as a tuple.
        """
        # Randomly pick first object
        obj, label = r.choice(zip(self.objects, self.labels))
        class_accuracies = []
        n_collects = 0 # Total number of object collections
        n_success = 0 # Number of successful collections
        for i in range(self.n_examples):
            fb = RichFeedback()
            fb.object = obj
            fb.label = label
            rospy.loginfo("Chose object {} with owner {} as example".
                          format(obj.id, label))
            rospy.sleep(0.1)
            self.feedback_pub.publish(fb)

            resp = self.classify()
            class_accuracies.append[self.evaluate(resp.objects)]

            # Select most likely unowned object to try as next example
            obj = max(resp.objects, key=lambda o: o.ownership[0])
            label = self.labels[self.objects.index(obj)]

            # Stop training if all objects are too likely to be owned
            if obj.ownership[0] < self.threshold:
                rospy.loginfo("All objects too likely to be owned.")
                rospy.loginfo("Stopping after {} examples.".format(i))
                n_collects = i
                break

            # Check if collection was correct
            predicted = obj.owners[obj.ownership.index(max(obj.ownership))]
            if predicted == label:
                n_success += 1

        # Compute accuracies
        collect_accuracy = float(n_success) / self.n_examples
        avg_class_accuracy = sum(class_accuracies) / len(class_accuracies)
        fin_class_accuracy = class_accuracies[-1]
        print("Collection accuracy: {} ({}/{})".
              format(collect_accuracy, n_success, self.n_examples))
        print("Average classification accuracy: {}".
              format(avg_class_accuracy))
        print("Final classification accuracy: {}".
              format(fin_class_accuracy))

        return (collect_accuracy, avg_class_accuracy, fin_class_accuracy)
        
    def evaluate(self, classified):
        """Evaluates and returns classifier accuracy."""
        predictions = []
        for o in classified:
            if o.is_avatar:
                continue
            # Prediction is the owner with maximum ownership probability
            predicted = o.owners[o.ownership.index(max(o.ownership))]
            predictions.append(predicted)

        # Assumes that predicted labels are in the same order as true labels
        correct = [(actual == predicted) for (actual, predicted) in
                   zip(self.labels, predictions)]
        n_correct = sum(correct)

        accuracy = float(n_correct) / self.n_objs
        print("Accuracy: {} ({}/{})".format(accuracy, n_correct, self.n_objs))

        return accuracy

    def reset(self):
        """Resets the classifier's interaction history."""
        msg = std_msgs.msg.Empty()
        self.reset_pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('object_tester')

    # Load parameters from launch file
    online = (rospy.get_param("online") if
              rospy.has_param("online") else True)
    trials = (rospy.get_param("trials") if
              rospy.has_param("trials") else 100)
    n_objs = (rospy.get_param("n_objs") if
              rospy.has_param("n_objs") else 10)
    n_avatars = (rospy.get_param("n_avatars") if
                 rospy.has_param("n_avatars") else 2)
    n_examples = (rospy.get_param("n_examples") if
                  rospy.has_param("n_examples") else 3)
    metrics = (rospy.get_param("metrics") if
               rospy.has_param("metrics") else
               ['color', 'position', 'proximity'])
    threshold = (rospy.get_param("collect_threshold") if
                 rospy.has_param("collect_threshold") else 0.2)

    tester = ObjectTester(metrics, n_objs, n_avatars, n_examples, threshold)
    results = []

    for t in range(trials):
        tester.generateEnvironment()
        rospy.wait_for_service('classify_objects')
        if online:
            results.append(tester.trainOnline())
        else:
            results.append(tester.trainOffline())
        tester.reset()

    # Transpose results and average them
    results = zip(*results)
    if online:
        avg_collect_accuracy = sum(results[0]) / len(results[0])
        avg_avg_class_accuracy = sum(results[1]) / len(results[1])
        avg_fin_class_accuracy = sum(results[2]) / len(results[2])
        print("Overall collection accuracy: {})".
              format(avg_collect_accuracy))
        print("Overall average classification accuracy: {}".
              format(avg_avg_class_accuracy))
        print("Overall final classification accuracy: {}".
              format(avg_fin_class_accuracy))
    else:
        avg_accuracy = sum(results[0]) / len(results[0])
        print("Overall accuracy: {})".format(avg_accuracy))
