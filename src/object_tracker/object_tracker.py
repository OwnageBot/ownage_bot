#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
import geometry_msgs.msg
from ownage_bot.msg import RichObject
from ownage_bot.msg import RichObjectArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self):
        self.latency = (rospy.get_param("tracker_latency") if
                        rospy.has_param("tracker_latency") else 0.1)
        self.object_db = dict()
        self.color_db = {} # Which colors have we seen?
        self.avatar_ids = (rospy.get_param("avatar_ids") if
                           rospy.has_param("avatar_ids") else [])
        self.landmark_ids = (rospy.get_param("landmark_ids") if
                             rospy.has_param("landmark_ids") else [])
        self.new_obj_pub = rospy.Publisher("new_object",
                                           UInt32, queue_size = 10)
        self.obj_db_pub = rospy.Publisher("object_db",
                                          RichObjectArray,
                                          queue_size = 10)

    def publishDb(self):
        rate = rospy.Rate(1/self.latency)
        while not rospy.is_shutdown():
          obj_arr = RichObjectArray()
          obj_arr.objects = self.object_db.values()
          self.obj_db_pub.publish(obj_arr)
          rate.sleep()

    def insertObject(self, marker):
        """Insert object into the database using marker information."""
        # Initialize fields that should be modified only once
        obj = RichObject()
        obj.id = marker.id
        obj.is_avatar = (marker.id in self.avatar_ids)
        obj.is_landmark = (str(marker.id) in self.landmark_ids)
        obj.forbiddenness = 0
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
                print "New object found!"
                print("Obj color: {}".format(obj.color))
                print("Color db: {}\n".format(self.color_db))
            elif ((rospy.get_rostime()-self.object_db[m.id].last_update) >
                  rospy.Duration(self.latency)):
                # Update object if update period has lapsed
                self.updateObject(m)

    def determineColor(self, msg, marker):
        """Determines color of the currently tracked object."""
        rospy.loginfo(" Determining Object Color\n")
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
        y_marker_pixels = range(y_min - 5, y_max + 5)
        x_marker_pixels = range(x_min - 5, x_max + 5)

        # Step through each pixel
        #for y in range(0, len(cv_image)):
             #for x in range(0,len(cv_image[y])):
        for y in range(y_min - 10, y_max + 10):
            for x in range(x_min - 10, x_max + 10):
                try:
                    r = cv_image[y][x][0]
                    g = cv_image[y][x][1]
                    b = cv_image[y][x][2]
                    # Only looks for pixels on peripharies of marker
                    if x in x_marker_pixels and y in y_marker_pixels:
                        pass

                    else:
                        obj_color.append(cv_image[y][x])


                except IndexError:
                    pass
        for p in obj_color:
            avg_r+= p[0]
            avg_g+= p[1]
            avg_b+= p[2]

        assert(len(obj_color) > 0)
        color = (avg_r/len(obj_color), avg_g/len(obj_color), avg_b/len(obj_color))
        return self.checkColorDatabase(color)

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


if __name__ == '__main__':
    rospy.init_node('object_tracker')
    objectTracker = ObjectTracker()
    # Published by aruco_ros
    rospy.Subscriber("/aruco_marker_publisher/markers",
                     MarkerArray, objectTracker.ARucoCallback)
    objectTracker.publishDb()
    rospy.spin()

