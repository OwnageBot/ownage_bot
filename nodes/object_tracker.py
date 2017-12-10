#!/usr/bin/env python
import rospy
import math
import cv2 as cv
import numpy as np
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger, TriggerResponse
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from baxter_core_msgs.msg import EndpointState, EndEffectorState
from human_robot_collaboration_msgs.msg import ArmState
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque, OrderedDict

class ObjectTracker:
    """Node that tracks and updates object properties."""

    def __init__(self):
        self.latency = rospy.get_param("~latency", 0.1)
        # Database of tentative objects
        self.tentative_db = dict()
        # Database of tracked objects
        self.object_db = dict()

        # State variables to track gripped objects
        self.cur_action = ""
        self.prev_action = ""
        self.gripped_id = -1

        self.avatar_ids = rospy.get_param("avatar_ids", [])

        # Servers for querying and modifying tracked object data
        self.lkp_obj_srv = rospy.Service("lookup_object", LookupObject,
                                         self.lookupObjectCb)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.listObjectsCb)
        self.rst_obj_srv = rospy.Service("reset_objects", Trigger,
                                          self.resetObjectsCb)

        # Publishers, subscribers, etc
        self.new_obj_pub = rospy.Publisher("new_object",
                                           ObjectMsg, queue_size = 10)
        self.owner_sub = rospy.Subscriber("owner_prediction",
                                              ObjectMsg, self.ownershipCb)

        # Subscribe to sensor data only if not running in simulation
        if rospy.get_param("simulation", False):
            self.initSimulated()
        else:
            self.initSensing()

    def initSimulated(self):
        """Initialize and subscribe to simulated world data."""
        # Set up service to lookup objects from simulator
        self.getSimulated = rospy.ServiceProxy("simulation/visible_objects",
                                             ListObjects)
        # Periodically lookup simulated objects
        rospy.Timer(rospy.Duration(self.latency),
                    lambda evt : self.updateSimulated())

    def updateSimulated(self):
        objs = self.getSimulated().objects
        for msg in objs:
            if msg.id not in self.object_db:
                new_obj = Object.fromMsg(msg)
                new_obj.ownership = dict()
                self.object_db[msg.id] = new_obj
                self.new_obj_pub.publish(new_obj.toMsg())
            else:
                self.object_db[msg.id].position = msg.position
                self.object_db[msg.id].orientation = msg.orientation
                self.object_db[msg.id].color = msg.color
                self.object_db[msg.id].categories = list(msg.categories)
        
    def initSensing(self):
        """Initialize and subscribe to real-world sensor data."""        
        # Subscribe to ARuCo markers and action state
        self.marker_sub = rospy.Subscriber("/aruco_marker_publisher/markers",
                                           MarkerArray, self.ARucoCb)
        self.action_sub = rospy.Subscriber("/action_provider/left/state",
                                           ArmState, self.actionCb)
        self.endpoint_sub = None

        # Margins around ARuco tag for color determination
        self.in_offset = rospy.get_param("~in_offset", 1)
        self.out_offset = rospy.get_param("~out_offset", 6)
        
        # Computer vision
        self.cv_bridge = CvBridge();
        # List of basic colors
        self.color_db = [(110, 45, 50), # Red
                         (40, 70, 60),  # Green
                         (35, 55, 115)] # Blue
        # List of basic colors in LAB color space
        self.lab_db = np.asarray(self.color_db, dtype="uint8")[... , None]
        self.lab_db = cv.cvtColor(np.swapaxes(self.lab_db, 1, 2),
                                  cv.COLOR_RGB2LAB)
        
    def insertARucoObject(self, marker):
        """Insert object into the database using marker information."""
        # Initialize fields that should be modified only once
        obj = Object()
        obj.id = marker.id
        obj.is_avatar = marker.id in self.avatar_ids
        self.object_db[marker.id] = obj
        # Initialize fields which are dynamically changing
        self.updateARucoObject(marker)
        return obj

    def updateARucoObject(self, marker):
        """Updates object database with given marker information."""
        # Assumes that marker.id is already in the database
        obj = self.object_db[marker.id]
        obj.last_update = rospy.get_rostime()
        obj.position = marker.pose.pose.position
        obj.orientation = marker.pose.pose.orientation
        # Proxmities are -1 if avatar cannot be found
        obj.proximities = [-1] * len(self.avatar_ids)
        for (i, k) in enumerate(self.avatar_ids):
            if k in self.object_db:
                avatar = self.object_db[k]
                obj.proximities[i] = objects.dist(obj, avatar)
        if marker.id in [2, 12, 19]:
            obj.color = "red"
        elif marker.id in [4, 5, 9, 10]:
            obj.color = "green"
        elif marker.id in [1, 3, 6]:
           obj.color = "blue"
        # One-time subscribe for image data
        # image_msg = rospy.wait_for_message(
        #      "/aruco_marker_publisher/result", Image)
        # obj.color = self.determineColor(image_msg, marker)

    def ARucoCb(self, msg):
        """Callback upon receiving list of markers from ARuco."""
        for m in msg.markers:
            # Check if object is already in database
            if m.id not in self.object_db:
                obj = self.insertARucoObject(m)
                # Publish that new object was found
                self.new_obj_pub.publish(obj.toMsg())
                rospy.loginfo("New object %s found!", obj.id)
            elif ((rospy.get_rostime()-self.object_db[m.id].last_update) >
                  rospy.Duration(self.latency)):
                # Update object if update period has lapsed
                self.updateARucoObject(m)

    def actionCb(self, msg):
        """Callback upon change in current action."""
        self.prev_action = self.cur_action
        self.cur_action = msg.action
        if self.cur_action == self.prev_action:
            return
        if self.cur_action == "get":
            # Start tracking object at endpoint
            self.gripped_id = int(msg.object)
            topic = "/robot/limb/left/endpoint_state"
            self.endpoint_sub = \
                rospy.Subscriber(topic, EndpointState, self.endpointCb)
        elif self.cur_action in ["put", "release"]:
            # Stop tracking object at endpoint
            self.gripped_id = -1
            self.endpoint_sub.unregister()
            self.endpoint_sub = None

    def endpointCb(self, msg):
        """Callback for endpoint state, used to track gripped objects."""
        if self.gripped_id < 0:
            return
        # Update gripped object's position in place
        obj = self.object_db[self.gripped_id]
        t_now = rospy.get_rostime()
        if (t_now - obj.last_update) > rospy.Duration(self.latency):
            topic = "/robot/end_effector/left_gripper"
            state = rospy.wait_for_message(topic, EndEffectorState)
            if state.gripping:
                obj.position = msg.pose.position

    def ownershipCb(self, msg):
        """Callback for owner prediction, updates database accordingly."""
        obj = Object.fromMsg(msg)
        self.object_db[obj.id].ownership = dict(obj.ownership)
                
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

    def lookupObjectCb(self, req):
        """ Returns properties of particular object"""
        if req.id in self.object_db:
            obj = self.object_db[req.id]
            return LookupObjectResponse(True, obj.toMsg())
        else:
            return LookupObjectResponse(False, ObjectMsg())

    def listObjectsCb(self, req):
        """Returns list of tracked objects"""
        return ListObjectsResponse([obj.toMsg() for
                                    obj in self.object_db.values()])

    def resetObjectsCb(self, req):
        """Clears the object databases."""
        self.tentative_db.clear()
        self.object_db.clear()
        return TriggerResponse(True, "")
    
if __name__ == '__main__':
    rospy.init_node('object_tracker')
    objectTracker = ObjectTracker()
    rospy.spin()

