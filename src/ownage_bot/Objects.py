import math
import copy
import rospy
import numpy as np
import matplotlib.path as mptPath
from ownage_bot.msg import RichObject
from geometry_msgs.msg import Point, Quaternion

class Object:
    """Represents objects in the workspace and their properties."""
            
    def __init__(self, msg=None):
        if msg is None:
            self.id = -1
            self.last_update = rospy.get_rostime()
            self.position = Point()
            self.orientation = Quaternion()
            self.proximities = [] # List of distances to avatars
            self.color = -1 # Red=0, Green=1, Blue=2
            self.ownership = dict() # Dictionary of ownership probabilities
            self.is_avatar = False # Whether object is an avatar
            self.is_landmark = False # Whether object is a landmark
        else:
            uncopyable = ["owners", "ownership"] 
            for k, v in msg.__dict__.items():
                if k in uncopyable:
                    continue
                if type(v) is tuple:
                    v = list(v)
                self.__dict__[k] = copy.deepcopy(v)
            self.ownership = dict(zip(obj_msg.owners, obj_msg.ownership))
            
    def asMessage(self):
        """Converts object to a ROS message."""
        msg = RichObject()
        uncopyable = ["ownership"] 
        for k, v in self.__dict__.items():
            if k in uncopyable:
                continue
            msg.__dict__[k] = copy.deepcopy(v)
        msg.owners = self.ownership.keys()
        msg.ownership = self.ownership.values()
        return msg

class Area:
    """Defines a 2D polygonal area."""
    def __init__(self, points):
        """Takes a list of tuples and stores them."""
        self.n_sides = len(points)
        self.points = np.array(points)
        self.path = mptPath.Path(self.points)
                                                    
def dist(obj1, obj2):
    """Calculates Euclidean distance between two objects."""
    p1 = obj1.position
    p2 = obj2.position
    diff = [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
    return math.sqrt(sum([d*d for d in diff]))

def inArea(obj, area):
    """Checks if object is located in area."""
    return area.path.contains_point((obj.position.x, obj.position.y))
