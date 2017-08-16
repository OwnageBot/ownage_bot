import math
import copy
import rospy
import numpy as np
import matplotlib.path as mptPath
from ownage_bot.msg import ObjectMsg
from ownage_bot.srv import LookupObject
from geometry_msgs.msg import Point, Quaternion

class Object:
    """Represents objects in the workspace and their properties."""

    _lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)
    _listObjects = rospy.ServiceProxy("list_objects", ListObjects)
    
    def __init__(self, id=-1, name="",
                 position=Point(), orientation=Quarterion(),
                 color=-1, owners=[],
                 is_avatar=False, is_landmark=False):
        self.id = id
        self.name = name
        self.last_update = rospy.get_rostime()
        self.position = position
        self.orientation = orientation
        self.proximities = [] # List of distances to avatars
        self.color = color # Red=0, Green=1, Blue=2
        self.ownership = dict() # Dictionary of ownership probabilities
        self.is_avatar = is_avatar # Whether object is an avatar
        self.is_landmark = is_landmark # Whether object is a landmark
        for o in owners:
            self.ownership[o] = 1.0

    def __eq__(self, other):
        """Objects are equal if their ids are."""
        if isinstance(other, self.__class__):
            return self.id == other.id
        return NotImplemented
        
    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash only the ID."""
        return hash(self.id)
        
    def toMsg(self):
        """Converts object to a ROS message."""
        msg = ObjectMsg()
        uncopyable = ["ownership"] 
        for k, v in self.__dict__.items():
            if k in uncopyable:
                continue
            setattr(msg, k, copy.deepcopy(v))
        msg.owners = self.ownership.keys()
        msg.ownership = self.ownership.values()
        return msg

    def toStr(self):
        """Minimal string representation of object."""
        return str(self.id)
    
    @classmethod
    def fromMsg(cls, msg):
        """Copy constructor from ObjectMsg."""
        obj = cls()
        if not isinstance(msg, ObjectMsg):
            raise TypeError("Copy constructor expects ObjectMsg.")
        copyable = ["id", "name", "last_update", "position", "orientation",
                    "proximities", "color", "is_avatar", "is_landmark"]
        for attr in copyable:
            val = getattr(msg, attr)
            if type(val) is tuple:
                val = list(val)
                self.__dict__[attr] = copy.deepcopy(val)
                self.ownership = dict(zip(msg.owners, msg.ownership))
            return

    @classmethod
    def fromStr(cls, s):
        """Convert ID string to Object by looking up database."""
        return cls.fromID(int(s))

    @classmethod
    def fromID(cls, oid):
        """Convert ID to Object by looking up database."""
        return cls.fromMsg(cls._lookupObject(oid).object)

    @classmethod
    def universe(cls):
        """Returns the set of all known Objects."""
        return set([cls.fromMsg(m) for m in cls._listObjects().objects])
    
class Agent:
    """Represents an agent that can own and act on objects."""
    def __init__(self, id=-1, name="", avatar_id=-1):
        self.id = id # Unique ID
        self.name = name # Human-readable name
        self.avatar_id = avatar_id # Object ID of avatar representing agent

    def __eq__(self, other):
        """Agents are equal if their ids are."""
        if isinstance(other, self.__class__):
            return self.id == other.id
        return NotImplemented
        
    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash only the ID."""
        return hash(self.id)

    def toStr(self):
        """Minimal string representation."""
        return str(self.id)

    @classmethod
    def fromStr(cls, s):
        """Convert to Agent from string."""
        return cls(int(s))

    @classmethod
    def universe(cls):
        """Returns the set of all known Agents."""
        return set([cls.fromMsg(m) for m in cls._listAgents().agents])
    
class Area:
    """Defines a 2D polygonal area."""
    def __init__(self, points, name=""):
        """Takes an iterable of 2-tuples and stores them."""
        self.name = name
        self.n_sides = len(points)
        self.points = tuple(points)
        self.path = mptPath.Path(np.array(self.points))

    def __eq__(self, other):
        """Areas are equal if their vertices are."""
        if isinstance(other, self.__class__):
            return self.points == other.points
        return NotImplemented
        
    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash only the vertices."""
        return hash(self.points)

    def toStr(self):
        """Minimal string representation."""
        return str(self.points)

    @classmethod
    def fromStr(cls, s):
        """Convert to Area from string."""
        return cls(eval(s))

    @classmethod
    def universe(cls):
        """Returns the set of all known Areas (defined in param server)."""
        area_set = set()
        area_names = rospy.get_param("area_names", [])
        for n in area_names:
            points = rospy.get_param(n + "/corners",)
            area_set.add(cls(points, name=n))
        return area_set
    
class Location:
    """Defines a location in space."""
    def __init__(self, point):
        """Takes a 3-tuple and stores it."""
        if len(point) != 3:
            raise ValueError("Position should be in 3 dimensions.")
        self.name = ""
        self.position = tuple(point)

    def __eq__(self, other):
        """Locations are equal if their vertices are."""
        if isinstance(other, self.__class__):
            return self.position == other.position
        return NotImplemented
        
    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash only the vertices."""
        return hash(self.position)

    def toStr(self):
        """Minimal string representation."""
        return str(self.position)

    @classmethod
    def fromStr(cls, s):
        """Convert to Location."""
        return cls(eval(s))
    
def dist(obj1, obj2):
    """Calculates Euclidean distance between two objects."""
    p1 = obj1.position
    p2 = obj2.position
    diff = [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
    return math.sqrt(sum([d*d for d in diff]))

def inArea(obj, area):
    """Checks if object is located in area."""
    return bool(area.path.contains_point((obj.position.x, obj.position.y)))
