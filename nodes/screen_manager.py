#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import *
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ScreenManager(object):
    """Displays system information on the Baxter screen."""

    def __init__(self):
        # Handle input and output from task manager node
        # self.task_sub = rospy.Subscriber("task_out", String,
        #                                  self.taskOutCb)
        # self.task_pub = rospy.Publisher("task_in", TaskMsg,
        #                                 queue_size=10)

        self.camera_sub = rospy.Subscriber("/aruco_marker_publisher/result",
                                           Image, self.cameraCb)
        self.screen_pub = rospy.Publisher("/robot/xdisplay",
                                           Image, queue_size=10)


    def cameraCb(self, msg):
        self.screen_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('screen_manager')
    screen_manager = ScreenManager()
    rospy.spin()
