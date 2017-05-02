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
    
    def __init__(self, metrics, n_objs, n_avatars, n_examples):
        """Constructs a tester with a corresponding test environment.

        metrics -- Cluster based on either 'color', 'position', 'proximity',
                   or some combination of them.
        n_objs -- Number of objects in the environment (excluding avatars)
        n_avatars -- Number of avatars
        n_examples -- Number of training examples, drawn from n_objs

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
        self.evaluate(resp.objects)

    def evaluate(self, classified):
        """Evaluates classifier accuracy."""
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

    # Load parametrs from launch file
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

    tester = ObjectTester(metrics, n_objs, n_avatars, n_examples)

    for t in range(trials):
        tester.generateEnvironment()
        rospy.wait_for_service('classify_objects')
        tester.trainOffline()
        tester.reset()
