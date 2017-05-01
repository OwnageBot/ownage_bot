#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from ownage_bot.srv import *
from ownage_bot.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import math

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self):
        self.latency = (rospy.get_param("tracker_latency") if
                        rospy.has_param("tracker_latency") else 0.1)
        self.object_db = dict()
        self.color_db = dict() # Which colors have we seen?
        # Margins around ARuco tag for color determination
        self.in_offset = (rospy.get_param("in_offset") if
                        rospy.has_param("in_offset") else 3)
        self.out_offset = (rospy.get_param("out_offset") if
                        rospy.has_param("out_offset") else 6)
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids") if
                             rospy.has_param("landmark_ids") else [])
        self.new_obj_pub = rospy.Publisher("new_object",
                                           UInt32, queue_size = 10)
        self.loc_obj_srv = rospy.Service("locate_object", LocateObject,
                                         self.locateObject)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.listObjects)

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
        if marker.id in [2, 12, 19]:
            obj.color = 0
        elif marker.id in [4, 5, 9, 10]:
            obj.color = 1
        elif marker.id in [1, 3, 6]:
            obj.color = 2
        # One-time subscribe for image data
        # image_msg = rospy.wait_for_message(
        #     "/aruco_marker_publisher/result", Image)
        # obj.color = self.determineColor(image_msg, marker)
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
                print "New object found!"
                print("Obj color: {}".format(obj.color))
                print("Color db: {}\n".format(self.color_db))
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

        # need to convert ROS images into OpenCV images in order to do analysis
        cv_image =  cv_image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
        obj_color = []  # will store pixels belonging ONLY to the object (hopefully)
        # These will store approx color of the object
        avg_r = 0
        avg_g = 0
        avg_b = 0

        # Get the coords of the corner of the marker
        # and sort x and y parts into diff lists
        corner_xs = [i.x for i in marker.corners]
        corner_ys = [i.y for i in marker.corners]

        # Get the coordinate range of the marker
        x_min = int(min(corner_xs))
        x_max = int(max(corner_xs))

        y_min = int(min(corner_ys))
        y_max = int(max(corner_ys))

        # Store the coords of the pixels within marker
        # bounding box
        y_marker_pixels = range(y_min - self.in_offset, y_max + self.in_offset)
        x_marker_pixels = range(x_min - self.in_offset, x_max + self.in_offset)

        # Step through each pixel
        for y in range(y_min - self.out_offset, y_max + self.out_offset):
            for x in range(x_min - self.out_offset, x_max + self.out_offset):
                try:
                    # Only looks for pixels on peripharies of marker
                    if x in x_marker_pixels and y in y_marker_pixels:
                        pass
                    else:
                        obj_color.append(cv_image[y][x])
                        # print(cv_image[y][x])

                except IndexError:
                    pass
        for p in obj_color:
            avg_r+= p[0]
            avg_g+= p[1]
            avg_b+= p[2]

        assert(len(obj_color) > 0)
        color = (avg_r/len(obj_color), avg_g/len(obj_color), avg_b/len(obj_color))
        color_index = color.index(max(color))

        if marker.id in self.color_db:
            self.color_db[marker.id].append(color_index)
        else:
            self.color_db[marker.id] = deque([color_index],20)

        mode = max(set(self.color_db[marker.id]), key=self.color_db[marker.id].count)

        rospy.logdebug("Obj: {}, Color: {}, Past colors: {}\n".format(marker.id, mode, self.color_db[marker.id]))
        return mode

    def checkColorDatabase(self, rgb_vals):
        """Categorizes objects based on colors, creates new names from novel colors."""
        # if there color bb is empty, add this color to the database
        if len(self.color_db) == 0:

            self.color_db[1] = rgb_vals
        # if the curr obj's color is within a certain range of a previouly seen color
        # than they are the same color
        else:
            for (k,v) in self.color_db.iteritems():
                for i in range(0,len(v)):
                    # We should play with this value more; it seems like
                    # there is a lot of noise with these cameras
                    if  abs(v[i] - rgb_vals[i]) >= 30:
                        break
                else:
                    return k
            # else, its a novel color and therefore should be added to the db.
            else:
                new_color = len(self.color_db) + 1
                self.color_db[new_color] = rgb_vals
                return new_color

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

