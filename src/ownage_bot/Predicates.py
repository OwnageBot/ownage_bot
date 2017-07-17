from ownage_bot import Objects

class Predicate:
    """Functions which apply to one or more objects, return true or false."""    
    def __init__(self, name="", argtypes=[]):
        self.name = name # Human-readable name
        self.n_args = len(argtypes) # Number of arguments
        self.argtypes = argtypes # List of argument types
        self.impl = lambda *args : True # Implementation of predicate

    def bind(self, arg, arg_loc):
        """Creates new predicate by binding argument at arg_id."""
        if arg_loc >= self.n_args:
            raise ValueError("Argument index is out of range.")
        if not isinstance(self.argtypes[arg_loc], arg):
            raise TypeError("Bound argument is the wrong type.")
        bound = self.__class__()
        bound.name = "%sBoundTo%s%iAt%i".format(
            self.name, arg.__class__.__name__, arg.id, arg_loc)
        bound.n_args = self.n_args - 1;
        bound.impl = lambda *args : \
            self.impl(*[a if (i == arg_loc) else arg
                        for (i,a) in enumerate(args)])
    
    def apply(self, *args):
        """Does type check, then applies implementation."""
        if len(args) != self.n_args:
            raise ValueError("Wrong number of arguments.")
        for t, a in zip(self.argtypes, args):
            if not isinstance(t, a):
                raise TypeError("Argument is the wrong type.")
        return self.impl(*args)
            
# List of pre-defined predicates
Red = Predicate("red", [Objects.Object])
Red.impl = lambda obj : (obj.color == 0)

Green = Predicate("green", [Objects.Object])
Green.impl = lambda obj : (obj.color == 1)

Blue = Predicate("blue", [Objects.Object])
Blue.impl = lambda obj : (obj.color == 2)

Near = Predicate("near", [Objects.Object, Objects.Object])
Near.impl = lambda obj1, obj2: Objects.dist(obj1, obj2) < 0.4

Owns = Predicate("owns", [int, Objects.Object])
Owns.impl = lambda agent, obj: obj.ownership[agent] > 0.8

IsOwned = Predicate("isOwned", [Objects.Object])
IsOwned.impl = lambda obj: any(map(lambda o:o>0.8, obj.ownership.iteritems()))

InArea = Predicate("inArea", [Objects.Object, Objects.Area])
InArea.impl = lambda obj, area: Objects.inArea(obj, area)
