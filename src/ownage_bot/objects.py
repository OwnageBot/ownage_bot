import math
import copy
import rospy
import time
import numpy as np
import matplotlib.path as mplPath
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point, Quaternion

class Constant(object):
    """Stores special constants with string representations."""
    def __init__(self, name):
        self.name = name

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.name == other.name
        return NotImplemented
        
    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        return hash(self.name)

    def toStr(self):
        return "_" + self.name + "_"

    def toPrint(self):
        return self.name
    
    @classmethod
    def fromStr(cls, s):
        return cls(s.strip('_'))

# Constant that represents existential quantifier
Any = Constant("any")

# Constant that represents empty / unbound argument
Nil = Constant("nil")
        
class Object(object):
    """Represents objects in the workspace and their properties."""

    _lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)
    _listObjects = rospy.ServiceProxy("list_objects", ListObjects)
    _universe_cache = []
    _last_cache_time = rospy.Time()
    _cache_latency = rospy.Duration(0.2)

    any_str = "something"
    nil_str = "it"
    
    def __init__(self, id=-1, name="",
                 position=Point(), orientation=Quaternion(),
                 speed=0.0, color="none", is_avatar=False,
                 owners=[], categories=[]):
        self.id = id
        self.name = name
        self.t_last_update = rospy.Time()
        self.position = position
        self.orientation = orientation
        self.speed = speed
        self.proximities = [] # List of distances to avatars
        self.color = color # Name of color
        self.ownership = dict() # Dictionary of ownership probabilities
        self.categories = dict() # Dictionary of category membership
        self.t_last_actions = dict() # Dictionary of last action times
        self.is_avatar = is_avatar # Whether object is an avatar
        for o in owners:
            self.ownership[o] = 1.0
        for c in categories:
            self.categories[c] = 1.0

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

    def copy(self):
        """Makes a copy of the Object."""
        obj = self.__class__()
        for k, v in self.__dict__.items():
            setattr(obj, k, copy.deepcopy(v))
        return obj
        
    def toMsg(self):
        """Converts Object to a ROS message."""
        msg = ObjectMsg()
        uncopyable = ["ownership", "categories", "t_last_actions"] 
        for k, v in self.__dict__.items():
            if k in uncopyable:
                continue
            setattr(msg, k, copy.deepcopy(v))
        msg.owners = self.ownership.keys()
        msg.ownership = self.ownership.values()
        msg.categories = self.categories.keys()
        msg.categoriness = self.categories.values()
        msg.actors = self.t_last_actions.keys()
        msg.t_last_actions = self.t_last_actions.values()
        return msg

    def toStr(self):
        """Minimal string representation of object."""
        return str(self.id)

    def toPrint(self, props=[]):
        """Human-readable string."""
        if len(props) == 0:
            return "object {}".format(self.id)
        out = []
        for p in props:
            if p == "id":
                s = "{:3d}".format(self.id)
            elif p == "t_last_update":
                secs = self.t_last_update.to_sec()
                s = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(secs))
            elif p == "color":
                s = "{:10}".format(self.color)
            elif p == "position":
                pos = self.position
                s = "({: 04.2f},{: 04.2f},{: 04.2f})".\
                    format(pos.x, pos.y, pos.z)
            elif p == "speed":
                s = "{: 04.2f}".format(self.speed)
            elif p == "ownership":
                s = ", ".join(["{:2d}: {: 04.2f}".format(k, v) for
                               k, v in self.ownership.iteritems()])
            elif p == "categories":
                s = ", ".join(["{:10}: {: 04.2f}".format(k, v) for
                               k, v in self.categories.iteritems()])
            elif p == "t_last_actions":
                t_fmt = "%H:%M:%S"
                times = {k: time.strftime(t_fmt, time.localtime(v.to_sec()))
                         for k, v in self.t_last_actions.iteritems()}
                s = ", ".join(["{:2d}: {}".format(k, v)
                               for k, v in times.iteritems()])
            else:
                s = ""
            out.append(s)
        return " ".join(out)

    def toSpeech(self):
        """String for speech synthesis."""
        return self.toPrint()
    
    @classmethod
    def fromMsg(cls, msg):
        """Copy constructor from ObjectMsg."""
        obj = cls()
        if not isinstance(msg, ObjectMsg):
            raise TypeError("Copy constructor expects ObjectMsg.")
        copyable = ["id", "name", "t_last_update", "position", "orientation",
                    "proximities", "color", "is_avatar"]
        for attr in copyable:
            val = getattr(msg, attr)
            if type(val) is tuple:
                val = list(val)
            obj.__dict__[attr] = copy.deepcopy(val)
        obj.ownership = dict(zip(msg.owners, msg.ownership))
        obj.categories = dict(zip(msg.categories, msg.categoriness))
        obj.t_last_actions = dict(zip(msg.actors, msg.t_last_actions))
        return obj

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
        """Returns a sorted list of all known Objects."""
        # Update cache if it has gotten old
        if (rospy.Time.now() - cls._last_cache_time) > cls._cache_latency:
            try:
                rospy.wait_for_service("list_objects",
                                       timeout=cls._cache_latency.to_sec())
                resp = cls._listObjects()
                cls._universe_cache =  [cls.fromMsg(m) for m in resp.objects]
                cls._universe_cache.sort(key = lambda x : x.toStr())
            except (rospy.ROSException, rospy.ServiceException):
                # Just return cache if service call could not be executed
                rospy.logwarn("Service error, returning cache instead...")
        return cls._universe_cache
    
class Agent(object):
    """Represents an agent that can own and act on objects."""

    _lookupAgent = rospy.ServiceProxy("lookup_agent", LookupAgent)
    _listAgents = rospy.ServiceProxy("list_agents", ListAgents)
    _universe_cache = []
    _last_cache_time = rospy.Time()
    _cache_latency = rospy.Duration(0.5)

    any_str = "somebody"
    nil_str = "them"
    
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

    def toPrint(self):
        """Human-readable string."""
        if self.name != "":
            return self.name
        return "agent {}".format(self.id)

    def toSpeech(self):
        """String for speech synthesis."""
        return self.toPrint()
    
    def toMsg(self):
        """Converts Agent to a ROS message."""
        msg = AgentMsg()
        for k, v in self.__dict__.items():
            setattr(msg, k, copy.deepcopy(v))
        return msg
    
    @classmethod
    def fromStr(cls, s):
        """Convert to Agent from string."""
        agents = cls.universe()
        for a in agents:
            if s.isdigit() and int(s) == a.id:
                return a
            if len(s) > 0 and s == a.name:
                return a
        if s.isdigit():
            return cls(id=int(s))
        else:
            return cls(name=s)

    @classmethod
    def fromMsg(cls, msg):
        """Copy constructor from AgentMsg."""
        agent = cls()
        if not isinstance(msg, AgentMsg):
            raise TypeError("Copy constructor expects AgentMsg.")
        copyable = ["id", "name", "avatar_id"]
        for attr in copyable:
            val = getattr(msg, attr)
            if type(val) is tuple:
                val = list(val)
            agent.__dict__[attr] = copy.deepcopy(val)
        return agent
    
    @classmethod
    def universe(cls):
        """Returns a sorted list of all known Agents."""
        # Update cache if it has gotten old
        if (rospy.Time.now() - cls._last_cache_time) > cls._cache_latency:
            try:
                rospy.wait_for_service("list_agents",
                                       timeout=cls._cache_latency.to_sec())
                resp = cls._listAgents()
                cls._universe_cache =  [cls.fromMsg(m) for m in resp.agents]
                cls._universe_cache.sort(key = lambda x : x.toStr())
            except (rospy.ROSException, rospy.ServiceException):
                # Just return cache if service call could not be executed
                rospy.logwarn("Service error, returning cache instead...")
        return cls._universe_cache
    
class Area(object):
    """Defines a 2D polygonal area."""

    any_str = "some"
    nil_str = "there"
    
    def __init__(self, points, name=""):
        """Takes an iterable of 2-tuples and stores them."""
        self.name = name
        self.n_sides = len(points)
        self.points = tuple([tuple(p[0:2]) for p in points])
        self.path = mplPath.Path(np.array(self.points))

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

    def toPrint(self):
        """Human-readable string."""
        if self.name != "":
            return self.name
        return self.toStr()

    def toSpeech(self):
        """String for speech synthesis."""
        return "the " + self.toPrint()
    
    @classmethod
    def fromStr(cls, s):
        """Convert to Area from string."""
        new = None
        try:
            points = eval(s)
            if type(points) in [tuple, list]:
                new = cls(points)
        except:
            pass
        # Try looking up in database
        areas = rospy.get_param("areas", dict())
        for k, v in areas.iteritems():
            a = cls(v["corners"], name=k)
            if (new == None and s == a.name) or (new == a):
                return a
        if new != None:
            return new
        raise ValueError("Could not construct or lookup Area.")

    @classmethod
    def universe(cls):
        """Returns a sorted list of all Areas (defined in param server)."""
        areas = rospy.get_param("areas", dict())
        l = [cls(v["corners"], name=k) for k, v in areas.iteritems()]
        return sorted(l, key = lambda x : x.toStr())
    
class Location(object):
    """Defines a location in space."""

    any_str = "somewhere"
    nil_str = "that location"
    
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

    def toPrint(self):
        """Human-readable string."""
        if self.name != "":
            return self.name
        return self.toStr()

    def toSpeech(self):
        """String for speech synthesis."""
        return self.toPrint()
    
    @classmethod
    def fromStr(cls, s):
        """Convert to Location."""
        return cls(eval(s))

class Category(object):
    """Defines a category of objects."""

    any_str = "in some category"
    nil_str = "in that category"
    
    def __init__(self, name):
        self.name = name # Human-readable name

    def __eq__(self, other):
        if type(other) == self.__class__:
            return self.name == other.name
        return NotImplemented
        
    def __ne__(self, other):
        if type(other) == self.__class__:
            return not self == other
        return NotImplemented

    def __hash__(self):
        return hash(self.name)

    def toStr(self):
        return self.name

    def toPrint(self):
        return self.name

    def toSpeech(self):
        """String for speech synthesis."""
        return "a " + self.toPrint()
    
    @classmethod
    def fromStr(cls, s):
        return cls(s)

    @classmethod
    def universe(cls):
        """Returns a sorted list of all known Categories."""
        names = rospy.get_param("categories", [])
        return sorted([cls(n) for n in names], key=lambda x : x.toStr())

class Color(Category):
    """Defines a color category."""

    any_str = "some color"
    nil_str = "that color"
    
    def __init__(self, name, hsv_range=[[0,0,0],[0,0,0]]):
        self.name = name # Human-readable name
        self.hsv_range = (tuple(hsv_range[0]), tuple(hsv_range[1]))

    def __eq__(self, other):
        if type(other) == self.__class__:
            return self.name == other.name
        return NotImplemented
        
    def __ne__(self, other):
        if type(other) == self.__class__:
            return not self == other
        return NotImplemented
        
    def __hash__(self):
        return hash((self.name, self.hsv_range))

    def toSpeech(self):
        """String for speech synthesis."""
        return self.name
    
    @classmethod
    def fromStr(cls, s):
        hsv_range = rospy.get_param("colors/" + s)
        return cls(s, hsv_range)

    @classmethod
    def universe(cls):
        """Returns a sorted list of all known Colors."""
        colors = rospy.get_param("colors", dict())
        l = [cls(name, hsv_range) for name, hsv_range in colors.items()]
        return sorted(l, key=lambda x : x.toStr())
    
def dist(p1, p2):
    """Calculates Euclidean distance between two points."""
    diff = [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
    return math.sqrt(sum([d*d for d in diff]))

def inArea(obj, area):
    """Checks if object is located in area."""
    return bool(area.path.contains_point((obj.position.x, obj.position.y)))
