#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from aruco_msgs.msg import MarkerArray
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RichObject:
    """Objects with additional properties."""
    def __init__(self):
        self.ARuco_id = -1;
        self.pose = geometry_msgs.msg.PoseWithCovariance()
        self.color = (0,0,0)
        self.last_update = rospy.Time()
        self.forbiddenness = 0
        self.is_avatar = 0
        self.is_landmark = 0

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self, update_period = 0.2):
        self.update_period = update_period # In seconds
        self.object_db = dict()
        self.avatar_ids = []
        self.landmark_ids = []
        self.object_color_db = {} # Which colors have we seen?
        self.new_obj_pub = rospy.Publisher("new_object", UInt32, queue_size = 10)

    def insertObject(self, marker):
        """Insert object into the database using marker information."""
        # Initialize fields that should be modified only once
        obj = RichObject()
        obj.ARuco_id = marker.id
        obj.is_avatar = (marker.id in self.avatar_ids)
        obj.is_landmark = (marker.id in self.landmark_ids)
        self.object_db[marker.id] = obj
        # Initialize fields which are dynamically changing
        self.updateObject(marker)
        return obj

    def updateObject(self, marker):
        """Updates object database with given marker information."""
        # Assumes that marker.id is already in the database
        obj = self.object_db[marker.id]
        obj.last_update = rospy.get_rostime()
        # This is like a subscriber except it unsubscribes after first message.
        image_msg = rospy.wait_for_message("/aruco_marker_publisher/result", Image);
        obj.color = self.determineColor(image_msg)
        obj.pose = marker.pose
        return obj

    def ARucoCallback(self, msg):
        """Callback upon receiving list of markers from ARuco."""
        for m in msg.markers:
            # Check if object is already in database
            if m.id not in self.object_db:
                obj = self.insertObject(m)
                self.new_obj_pub.publish(m.id) # Publish that new object was found
                rospy.loginfo("New object %s found!", m.id)
                print "New object found!"
                print("Obj color: {}".format(self.curr_obj.color))
                print("Color db: {}\n".format(self.object_color_db))
            else if ((rospy.get_rostime()-self.object_db[m.id].last_update) >
                     self.update_period):
                # Update object if update period has lapsed
                self.updateObject(m)

    def determineColor(self, msg):
        """Determines color of the currently tracked object."""
        rospy.loginfo(" Determining Object Color\n")
        # need to convert ROS images into OpenCV images in order to do analysis
        cv_image =  cv_image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
        obj_color = []  # will store pixels belonging ONLY to the object (hopefully)
        # These will store approx color of the object
        avg_r = 0 
        avg_g = 0
        avg_b = 0

        # Step through each pixel
        for y in range(0, len(cv_image)):
            for x in range(0,len(cv_image[y])):
                r = cv_image[y][x][0]
                g = cv_image[y][x][1]
                b = cv_image[y][x][2]

                # detects the bounding box of the QR code, which is blue
                if r <10 and g < 10 and b > 200:
                    #print(x,y)
                # Given that the aruco tag can be in any orientation we determine
                # which pixels belong to the object w a simple heuristic:
                # pixels to the left and close to the first bounding box pixel
                # in a given row likely belongs to the object
                    try:
                        obj_pixel = cv_image[y][x-3] # 2 is just a guess, feel free to change
                        # Another heurisitc: we dont want to count the red and green axes drawn by aruco
                        # either
                        if sum(obj_pixel) != 255:
                            #rospy.loginfo(" approx color of obj: {}".format(obj_pixel))
                            obj_color.append(obj_pixel)
                        else:
                            pass

                    except IndexError:
                        pass
                    # we only care about the first blue pixel we see in a given row y
                    break

        for p in obj_color:
            avg_r+= p[0]
            avg_g+= p[1]
            avg_b+= p[2]
        
        assert(len(obj_color) > 0)
        color =  (avg_r/len(obj_color), avg_g/len(obj_color), avg_b/len(obj_color))
        return self.checkColorDatabase(color)

    def checkColorDatabase(self, rgb_vals):
        """Categorizes objects based on colors, creates new names from novel colors."""
        # if there color bb is empty, add this color to the database
        if len(self.object_color_db) == 0:

            self.object_color_db["Color 1"] = rgb_vals
        # if the curr obj's color is within a certain range of a previouly seen color
        # than they are the same color
        else: 
            for (k,v) in self.object_color_db.iteritems():
                for i in range(0,len(v)):
                    # We should play with this value more; it seems like
                    # there is a lot of noise with these cameras
                    if  abs(v[i] - rgb_vals[i]) >= 10:
                        break
                else: 
                    return k
            # else, its a novel color and therefore should be added to the db.
            else:
                new_color = "Color {}".format(len(self.object_color_db) + 1)
                self.object_color_db[new_color] = rgb_vals 
                return new_color
            

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    objectTracker = ObjectTracker()
    # published by aruco_ros
    rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, objectTracker.ARucoCallback)
    rospy.spin()

