import math
import copy
import rospy
from ownage_bot.msg import RichObject
from geometry_msgs.msg import Point, Quaternion

class Object:
    """Represents objects in the workspace and their properties."""
            
    def __init__(self, msg=None):
        if msg is None:
            self.id = -1
            self.last_update = rospy.get_rostime()
            self.position = Point()
            self.orientation = Quarternion()
            self.proximities = [] # List of distances to avatars
            self.color = -1 # Red=0, Green=1, Blue=2
            self.ownership = dict() # Dictionary of ownership probabilities
            self.is_avatar = False # Whether object is an avatar
            self.is_landmark = False # Whether object is a landmark
        else:
            uncopyable = ["owners, "ownership"] 
            for k, v in msg.__dict__.items():
                if k in uncopyable:
                    continue
                if type(v) is tuple:
                    v = list(v)
                self.__dict__[k] = copy.deepcopy(v)
            self.ownership = dict(zip(obj_msg.owners, obj_msg.ownership))
            
    def asMessage(self):
        msg = RichObject()
        uncopyable = ["ownership"] 
        for k, v in self.__dict__.items():
            if k in uncopyable:
                continue
            msg.__dict__[k] = copy.deepcopy(v)
        msg.owners = self.ownership.keys()
        msg.ownership = self.ownership.values()
        return msg

def dist(obj1, obj2):
    """Calculates Euclidean distance between two objects."""
    p1 = obj1.position
    p2 = obj2.position
    diff = [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
    return math.sqrt(sum([d*d for d in diff]))
