#!/usr/bin/env python
import rospy
import math
import cv2 as cv
import numpy as np
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from ownage_bot.srv import *
from ownage_bot.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque, OrderedDict

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self):
        self.latency = (rospy.get_param("tracker_latency") if
                        rospy.has_param("tracker_latency") else 0.1)
        self.object_db = dict()

        # Margins around ARuco tag for color determination
        self.in_offset = (rospy.get_param("in_offset") if
                          rospy.has_param("in_offset") else 1)
        self.out_offset = (rospy.get_param("out_offset") if
                           rospy.has_param("out_offset") else 6)

        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids") if
                             rospy.has_param("landmark_ids") else [])

        # Publishers and servers
        self.new_obj_pub = rospy.Publisher("new_object",
                                           UInt32, queue_size = 10)
        self.loc_obj_srv = rospy.Service("locate_object", LocateObject,
                                         self.locateObject)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.listObjects)
        
        # Computer vision
        self.cv_bridge = CvBridge();
        # List of basic colors
        self.color_db = [(110, 45, 50), # Red
                         (40, 70, 60),  # Green
                         (35, 55, 115)] # Blue
        # List of basic colors in LAB color space
        self.lab_db = np.asarray(self.color_db, dtype="uint8")[... , None]
        self.lab_db = cv.cvtColor(np.swapaxes(self.lab_db, 1, 2), cv.COLOR_RGB2LAB)
        cv.namedWindow("Mask")

    def insertObject(self, marker):
        """Insert object into the database using marker information."""
        # Initialize fields that should be modified only once
        obj = RichObject()
        obj.id = marker.id
        obj.is_avatar = (marker.id in self.avatar_ids)
        obj.is_landmark = (marker.id in self.landmark_ids)
        obj.owners = [0] # Default to unowned
        obj.ownership = [1.0] # With probability 1
        self.object_db[marker.id] = obj
        # Initialize fields which are dynamically changing
        self.updateObject(marker)
        return obj

    def updateObject(self, marker):
        """Updates object database with given marker information."""
        # Assumes that marker.id is already in the database
        obj = self.object_db[marker.id]
        obj.last_update = rospy.get_rostime()
        obj.pose = marker.pose
        # Proxmities are -1 if avatar cannot be found
        obj.proximities = [-1] * len(self.avatar_ids)
        for (i, k) in enumerate(self.avatar_ids):
            if k in self.object_db:
                avatar = self.object_db[k]
                obj.proximities[i] = self.computeProximity(obj, avatar)
        # if marker.id in [2, 12, 19]:
        #     obj.color = 0
        # elif marker.id in [4, 5, 9, 10]:
        #     obj.color = 1
        # elif marker.id in [1, 3, 6]:
        #    obj.color = 2
        # One-time subscribe for image data
        image_msg = rospy.wait_for_message(
             "/aruco_marker_publisher/result", Image)
        obj.color = self.determineColor(image_msg, marker)
        return obj

    def ARucoCallback(self, msg):
        """Callback upon receiving list of markers from ARuco."""
        for m in msg.markers:
            # Check if object is already in database
            if m.id not in self.object_db:
                obj = self.insertObject(m)
                # Publish that new object was found
                self.new_obj_pub.publish(m.id)
                rospy.loginfo("New object %s found!", m.id)
            elif ((rospy.get_rostime()-self.object_db[m.id].last_update) >
                  rospy.Duration(self.latency)):
                # Update object if update period has lapsed
                self.updateObject(m)

    def computeProximity(self, obj1, obj2):
        """Computes 2D Euclidean distance between two objects."""
        p1 = obj1.pose.pose.position
        p2 = obj2.pose.pose.position
        return math.sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

    def determineColor(self, msg, marker):
        """Determines color of the currently tracked object."""
        rospy.logdebug(" Determining Object Color\n")

        # Convert to OpenCV image (stored as Numpy array)
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        
        # Draw mask on outer border of ARuco marker
        mask = np.zeros(cv_image.shape[:2], dtype="uint8")
        contour = np.array([[c.x,c.y] for c in marker.corners], dtype="int32");
        cv.drawContours(mask, [contour], -1, 255, 2*self.out_offset)
        cv.drawContours(mask, [contour], -1, 0, 2*self.in_offset)
        cv.drawContours(mask, [contour], -1, 0, -1)

        # Compute mean color in LAB color space
        lab_image = cv.cvtColor(cv_image, cv.COLOR_RGB2LAB)
        mean = cv.mean(lab_image, mask=mask)[:3]

        # Find distance to basic colors, return index of closest color
        minDist = np.inf
        colorId = len(self.color_db)
        for (i, c) in enumerate(self.lab_db):
            dist = sum(np.square(c[0]-mean))
            if dist < minDist:
                minDist = dist
                colorId = i
 
        return colorId

    def locateObject(self, req):
        """ Service callback: returns position of particular object"""
        if req.id in self.object_db:
            p = self.object_db[req.id].pose.pose
            return LocateObjectResponse(True, p)
        else:
            return LocateObjectResponse(False, geometry_msgs.msg.Pose())

    def listObjects(self, req):
        """ Service callback: returns list of tracked objects"""
        return ListObjectsResponse(self.object_db.values())

if __name__ == '__main__':
    rospy.init_node('object_tracker')

    objectTracker = ObjectTracker()

    # Published by aruco_ros
    rospy.Subscriber("/aruco_marker_publisher/markers",
                     MarkerArray, objectTracker.ARucoCallback)

    rospy.spin()

