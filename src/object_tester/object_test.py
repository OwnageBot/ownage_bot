#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from ownage_bot.msg import *
import math
import random as r


class ObjectTester():
    def __init__(self, num_landmarks, num_avatars, num_objs):

        self.num_landmarks = num_landmarks # Not used yet
        self.num_avatars = num_avatars
        self.num_objs = num_objs

        self.pub_fb = rospy.Publisher("feedback",
                                        RichFeedback)
        self.pub_objs = rospy.Publisher("object_db",
                                        RichObjectArray)

    def generateClusters(self):
        """Randomly generates avatars and randomly clusters
        red Objects around them """

        num_landmarks = 0

        fb_array = []
        avatars = []
        obj_array = RichObjectArray()

        for i in range(self.num_avatars):
            av = RichObject()
            av.id = i
            av.is_avatar = True

            av.pose.pose.x = r.uniform(-10.0, 10.0)
            av.pose.pose.y = r.uniform(-10.0, 10.0)
            av.pose.pose.z = 2.0

            avatars.append(av)
            obj_array.append(av)
            
        for i in range(self.num_objs):

            ids = range(self.num_avatars,self.num_avatars + self.num_objs)
            obj = RichObject()
            rand_offset = r.uniform(-2.0, 2.0)

            obj.id = ids.pop()
            obj.color = 0

            av = r.choice(avatars)

            obj.pose.pose.x = av.pose.pose.x + rand_offset
            obj.pose.pose.y = av.pose.pose.y + rand_offset
            obj.pose.pose.z = av.pose.pose.z

            obj.proximities = [self.dist(av, obj) for i in avatars]

            obj.owners = [0]
            obj.ownership = [1.0]

            obj.is_avatar = False

            if r.Random() > 70:
                fb = RichFeedback()
                fb.label = av

                obj.is_landmark = True
                fb.object = obj

                num_landmarks += 1
                print("There are {} landmarks!\n".format(num_landmarks))
                self.pub_fb(fb)

            obj_array.append(obj)

        self.pub_objs(obj_array)

    def dist(self, av, obj):
        av_x = av.pose.pose.x
        av_y = av.pose.pose.y
        av_z = av.pose.pose.z

        obj_x = obj.pose.pose.x
        obj_y = obj.pose.pose.y
        obj_z = obj.pose.pose.z

        sqr = lambda x: x * x

        return (sqr(av_x - obj_x) + sqr(av_y - obj_y) + sqr(av_z - obj_z))

if __name__ == '__main__':
    rospy.init_node('object_tester')

    tester = ObjectTester(0, 2,20)
    tester.generateClusters()

    rospy.spin()
