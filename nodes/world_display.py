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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class WorldDisplay(object):
    """Graphical display of tracked objects and their properties."""
    
    def __init__(self):
        # Display width and height in pixels
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 400)
        self.fps = rospy.get_param("~fps", 25)

        # CvBridge instance for ROS <-> OpenCV conversion
        self.cv_bridge = CvBridge()

        # Display buffer
        self.disp_buf = np.zeros((self.height, self.width, 3), np.uint8)

        # 2D physical location that top-left of display maps to
        self.origin = rospy.get_param("~origin", (0.2, -0.4))
        # Pixel-to-meter scaling ratio
        self.scaling = rospy.get_param("~scaling", 480)
        # Flag to swap X and Y axes (set to True to match Baxter camera output)
        self.swap_xy = rospy.get_param("~swap_xy", True)
        # Flag to flip X and Y axes (set to True to match Baxter camera output)
        self.flip_xy = rospy.get_param("~flip_xy", True)
        # Flag to color objects by ownership probability
        self.owner_heatmap = rospy.get_param("~owner_heatmap", -1)
        # Radius in pixels of the circular object markers
        self.obj_radius = rospy.get_param("~obj_radius", 8)
        # Mapping from color names to RGB values
        self.color_db = {"red": (255,0,0),
                         "green": (0,255,0),
                         "blue": (0,0,255)}

        # Publisher for images
        self.display_pub = rospy.Publisher("world_display", Image, queue_size=10)
        # Update display at refresh rate
        rospy.Timer(rospy.Duration(1.0/self.fps), self.updateDisplay)

    def drawObject(self, obj):
        """Draws object on display buffer."""
        # Convert color name to RGB value
        color = self.color_db.get(obj.color, (255, 255, 255))
        if self.owner_heatmap < 0:
            # Color by object color
            obj_color = color
            txt_color = (255, 255, 255)
        else:
            # Color by ownership probability
            own_prob = obj.ownership.get(self.owner_heatmap, 0.5)
            obj_color = (int(own_prob * 255), ) * 3
            txt_color = color
        # Transform from actual coordinates to pixels
        x = int(self.scaling * (obj.position.x - self.origin[0]))
        y = int(self.scaling * (obj.position.y - self.origin[1]))
        if self.swap_xy:
            x, y = y, x
        if self.flip_xy:
            x, y = self.width - x, self.height - y
        # Draw filled circle at pixel coordinates
        cv2.circle(self.disp_buf, (x, y), self.obj_radius, obj_color, -1)
        # Draw object ID on top of circle
        cv2.putText(self.disp_buf, str(obj.id), (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color, 2, cv2.CV_AA)


    def drawDisplay(self):
        """Draws display buffer."""
        # Clear display to grey
        self.disp_buf.fill(127)
        # Draw origin and scaling
        cv2.putText(self.disp_buf, "ORIGIN: {}".format(self.origin), (24, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.CV_AA)
        cv2.putText(self.disp_buf, "SCALING: {}".format(self.scaling), (24, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.CV_AA)
        # Draw all objects
        objs = list(Object.universe())
        for obj in objs:
            self.drawObject(obj)

    def updateDisplay(self, event):
        """Publish display buffer as Image msg."""
        self.drawDisplay()
        msg = self.cv_bridge.cv2_to_imgmsg(self.disp_buf, encoding="rgb8")
        self.display_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('world_display')
    world_display = WorldDisplay()
    rospy.spin()
