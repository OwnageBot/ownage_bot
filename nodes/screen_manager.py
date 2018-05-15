#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import *
from geometry_msgs.msg import Point
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ScreenManager(object):
    """Displays system information on the Baxter screen."""

    def __init__(self):
        # Screen width and height in pixels
        self.width = rospy.get_param("~width", 1024)
        self.height = rospy.get_param("~height", 600)
        self.fps = rospy.get_param("~fps", 25)

        # CvBridge instance for ROS <-> OpenCV conversion
        self.cv_bridge = CvBridge()

        # Image buffers
        self.screen_buf = np.zeros((self.height, self.width, 3), np.uint8)
        self.status_buf = np.zeros((self.height/4, self.width, 3), np.uint8)
        self.camera_buf = None

        # Text buffers
        self.speech_buf = ""
        self.cmd_buf = ""
        
        # Subscribers and publishers
        self.cmd_sub = rospy.Subscriber("last_cmd", String,
                                          self.cmdInputCb)
        self.speech_sub = rospy.Subscriber("last_utt", String,
                                          self.speechInputCb)
        self.camera_sub = rospy.Subscriber("/aruco_marker_publisher/result",
                                           Image, self.cameraCb)
        self.screen_pub = rospy.Publisher("/robot/xdisplay",
                                           Image, queue_size=10)

        # Update screen at refresh rate
        rospy.Timer(rospy.Duration(1.0/self.fps), self.updateScreen)

    def cmdInputCb(self, msg):
        """Sets cmd buffer to received string."""
        self.cmd_buf = msg.data

    def speechInputCb(self, msg):
        """Sets speech buffer to received string."""
        self.speech_buf = msg.data

    def cameraCb(self, msg):
        """Sets camera buffer to received image."""
        self.camera_buf = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")

    def drawStatus(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1
        color = (255, 255, 255)
        thickness = 3
        line_type = cv2.CV_AA

        self.status_buf.fill(0)
        # Draw last text command
        cv2.putText(self.status_buf, "CMD: {}".format(self.cmd_buf), 
                    (20, 40), font, scale, color, thickness, line_type)
        # Draw last speech utterance
        cv2.putText(self.status_buf, "SPEECH: {}".format(self.speech_buf), 
                    (20, 80), font, scale, color, thickness, line_type)

    def drawScreen(self):
        self.screen_buf.fill(0)

        # Draw status bar and copy to screen buffer
        self.drawStatus()
        status_w = self.status_buf.shape[1]
        status_h = self.status_buf.shape[0]
        self.screen_buf[0:status_h, 0:status_w] = self.status_buf

        # Copy camera image (if present) to screen buffer
        if self.camera_buf is None:
            return
        cam_w = self.camera_buf.shape[1]
        cam_h = self.camera_buf.shape[0]
        cam_x = int((self.width - cam_w) / 2)
        cam_y = int((self.height-status_h-cam_h) / 2) + status_h
        self.screen_buf[cam_y:cam_y+cam_h, cam_x:cam_x+cam_w] = self.camera_buf
    
    def updateScreen(self, event):
        self.drawScreen()
        msg = self.cv_bridge.cv2_to_imgmsg(self.screen_buf, encoding="rgb8")
        self.screen_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('screen_manager')
    screen_manager = ScreenManager()
    rospy.spin()
