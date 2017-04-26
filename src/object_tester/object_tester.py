#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from ownage_bot.msg import *
from ownage_bot.srv import *
import math
import random as r

# color, position, or proximity
METRIC = "proximity"

class ObjectTester():
    def __init__(self, num_landmarks, num_avatars, num_objs):

        self.num_landmarks = num_landmarks # Not used yet
        self.num_avatars = num_avatars
        self.num_objs = num_objs

        self.object_db = []
        self.labels = []

        self.pub_fb = rospy.Publisher("feedback",
                                      RichFeedback,
                                      queue_size=10)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.listObjects)

    def generateClusters(self):
        """Randomly generates avatars and object clusters."""

        fb_array = []
        avatars = []
        centers = []
        obj_array = []

        home = geometry_msgs.msg.Point()
        home.x = 0.0
        home.y = 0.0
        home.z = 2.0

        for i in range(self.num_avatars):
            av = RichObject()
            av.id = 100 + i
            av.is_avatar = True

            av.pose.pose.position.x = r.uniform(-1.0, 1.0)
            av.pose.pose.position.y = r.uniform(-1.0, 1.0)
            av.pose.pose.position.z = 2.0

            avatars.append(av)
            obj_array.append(av)

        for i in range(self.num_avatars + 1):
            c = geometry_msgs.msg.Point()

            c.x = r.uniform(-1.0, 1.0)
            c.y = r.uniform(-1.0, 1.0)
            c.z = 2.0

            centers.append(c)

        ids = range(10, 10 + self.num_objs)

        for i in range(self.num_objs):

            obj = RichObject()

            obj.id = ids.pop()

            category = r.choice(range(len(avatars) + 1))
            label = 0 if category == 0 else avatars[category-1].id

            if METRIC == "color":
                obj.pose.pose.position.x = r.uniform(-1.0, 1.0)
                obj.pose.pose.position.y = r.uniform(-1.0, 1.0)
                obj.pose.pose.position.z = r.uniform(-1.0, 1.0)

                obj.color = category

            elif METRIC == "proximity":
                rand_offset = r.uniform(-0.5, 0.5)
                avp = (home if category == 0 else
                       avatars[category-1].pose.pose.position)

                obj.pose.pose.position.x = avp.x + rand_offset
                obj.pose.pose.position.y = avp.y + rand_offset
                obj.pose.pose.position.z = avp.z

                obj.color = 0

            elif METRIC == "position":
                rand_offset = r.uniform(-0.5, 0.5)
                c = centers[category]

                obj.pose.pose.position.x = c.x + rand_offset
                obj.pose.pose.position.y = c.y + rand_offset
                obj.pose.pose.position.z = c.z

                obj.color = 0

            obj.proximities = [self.dist(j, obj) for j in avatars]

            obj.owners = [0]
            obj.ownership = [1.0]

            obj.is_avatar = False

            if i < self.num_landmarks:

                fb = RichFeedback()
                fb.object = obj
                fb.label = label

                rospy.loginfo("There are {} landmarks!\n".format(i + 1))
                rospy.sleep(.1)
                self.pub_fb.publish(fb)

            obj_array.append(obj)
            self.labels.append(label)

        self.object_db = obj_array

    def listObjects(self, req):
        """ Service callback: returns list of tracked objects"""
        return ListObjectsResponse(self.object_db)

    def trainClassifier(self):
        rospy.wait_for_service('classifyObjects')
        try:
            classify = rospy.ServiceProxy("classifyObjects", ListObjects)
            resp = classify()

            predictions = []

            for o in resp.objects:
                if o.is_avatar:
                    continue
                predicted = o.owners[o.ownership.index(max(o.ownership))]
                predictions.append(predicted)

            correct = 0

            for (actual, predicted) in zip(self.labels, predictions):
                print actual, predicted
                if actual == predicted:
                    correct += 1

            print("TOTAL FRACTION CORRECT: {}".format(correct /float(self.num_objs)))

        except rospy.ServiceException,e:
            rospy.logerr("Classify Objects srv failed! {}".format(e))



    def dist(self, av, obj):
        av_x = av.pose.pose.position.x
        av_y = av.pose.pose.position.y
        av_z = av.pose.pose.position.z

        obj_x = obj.pose.pose.position.x
        obj_y = obj.pose.pose.position.y
        obj_z = obj.pose.pose.position.z

        sqr = lambda x: x * x

        return float(sqr(av_x - obj_x) + sqr(av_y - obj_y) + sqr(av_z - obj_z))

if __name__ == '__main__':
    rospy.init_node('object_tester')

    tester = ObjectTester(3, 2, 10)
    tester.generateClusters()

    tester.trainClassifier()

    rospy.spin()
