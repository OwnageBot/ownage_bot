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
        self.local_pose = geometry_msgs.msg.PoseWithCovariance()
        self.color = (0,0,0)
        self.last_movement = 0
        self.forbiddenness = 0
        self.is_avatar = 0
        self.is_landmark = 0

class ObjectTracker:
    """A class for tracking objects."""

    def __init__(self):
        self.object_db = []
        self.avatar_ids = []
        self.landmark_ids = []
        self.object_color_db = {} # which colors have we seen?
        self.curr_obj = None # Whats the object currently being tracked?
        self.new_obj_pub = rospy.Publisher("new_object", UInt32, queue_size = 10)


    def ARucoCallback(self, msg):
        for m in msg.markers:
            # Check if object is already in database
            if m.id not in [o.ARuco_id for o in self.object_db]:
                self.curr_obj = RichObject()
                # This is like a subscriber except it unsubscribes after first message.
                self.curr_obj.color = self.determineColor(rospy.wait_for_message("/aruco_marker_publisher/result", Image))
                self.curr_obj.ARuco_id = m.id
                self.curr_obj.local_pose = m.pose # Actually PoseWithCovariance
                self.curr_obj.is_avatar = (m.id in self.avatar_ids)
                self.curr_obj.is_landmark = (m.id in self.landmark_ids)
                self.object_db.append(self.curr_obj)
                self.new_obj_pub.publish(m.id) # Publish that new object was found
                rospy.loginfo("New object %s found!", m.id)
                print "New object found!"
                print("Obj color: {}".format(self.curr_obj.color))
                print("Color db: {}\n".format(self.object_color_db))

    def determineColor(self, msg):
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



    # This will categorize objects based on colors it has alreadys seen,
    # and will create novel color names if novel colors are encountered.
    def checkColorDatabase(self, rgb_vals):
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

