import objects
from objects import Object, Agent, Area

class Predicate:
    """Functions which apply to one or more objects, return true or false."""    
    def __init__(self, name="", argtypes=[]):
        self.name = name # Human-readable name
        self.n_args = len(argtypes) # Number of arguments
        self.argtypes = argtypes # List of argument types
        self.parent = self # Parent predicate is self if not derived
        self.binding = (-1, -1) # (loc, arg) tuple if bound
        self._apply = lambda *args : True # Implementation of predicate

    def __hash__(self):
        """Hash using only name and argtypes."""
        return hash(tuple(self.name, tuple(self.argtypes)))
        
    def bind(self, arg, arg_loc):
        """Derive new predicate by binding argument at arg_id."""
        if arg_loc >= self.n_args:
            raise ValueError("Argument index is out of range.")
        if not isinstance(arg, self.argtypes[arg_loc]):
            raise TypeError("Bound argument is the wrong type.")
        bound = self.__class__()
        bound.name = "{}BoundTo{}{}At{}".format(
            self.name, arg.__class__.__name__, arg.id, arg_loc)
        bound.n_args = self.n_args - 1;
        bound.argtypes = [t for (i,t) in enumerate(self.argtypes) if
                          i != arg_loc]
        bound.parent = self
        bound.binding = (arg_loc, arg)
        bound._apply = lambda *args : \
            self._apply(*[arg if (i == arg_loc) else a
                        for (i,a) in enumerate(args)])
    
    def apply(self, *args):
        """Does type check, then applies implementation."""
        if len(args) != self.n_args:
            raise ValueError("Wrong number of arguments.")
        for t, a in zip(self.argtypes, args):
            if not isinstance(a, t):
                raise TypeError("Argument is the wrong type.")
        return self._apply(*args)
            
# List of pre-defined predicates
Red = Predicate("red", [Object])
Red._apply = lambda obj : (obj.color == 0)

Green = Predicate("green", [Object])
Green._apply = lambda obj : (obj.color == 1)

Blue = Predicate("blue", [Object])
Blue._apply = lambda obj : (obj.color == 2)

Near = Predicate("near", [Object, Object])
Near._apply = lambda obj1, obj2: objects.dist(obj1, obj2) < 0.4

OwnedBy = Predicate("owns", [Object, Agent])
OwnedBy._apply = lambda obj, agent: obj.ownership[agent.id] > 0.8

IsOwned = Predicate("isOwned", [Object])
IsOwned._apply = lambda obj: any(map(lambda o:o>0.8,
                                     obj.ownership.iteritems()))

InArea = Predicate("inArea", [Object, Area])
InArea._apply = lambda obj, area: objects.inArea(obj, area)

# List of available predicates for each robotic platform
if rospy.get_param("platform", "baxter") == "baxter":
    # Only Baxter is currently supported
    db = [Red, Green, Blue, Near, OwnedBy, IsOwned, InArea]
else:
    db = []
