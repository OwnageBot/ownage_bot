from ownage_bot import Objects

class Predicate:
    """Functions which apply to one or more objects, return true or false."""    
    def __init__(self, name="", n_args=0):
        self.name = name # Human-readable name
        self.n_args = n_args;
        self.apply = lambda *args : True # Function that applies predicate

    def bind(self, arg, arg_loc):
        """Creates new predicate by binding argument at arg_id."""
        if arg_loc >= n_args:
            raise ValueError("Argument index is out of range.")
        if not isinstance(Objects.Object, arg):
            raise TypeError("Bound argument is not an object.")
        bound = self.__class__()
        bound.name = "%sBoundToObj%iAt%i"(self.name, arg.id, arg_loc)
        bound.n_args = self.n_args - 1;
        bound.apply = lambda *args : \
            self.apply(*[a if (i == arg_loc) else arg
                         for (i,a) in enumerate(args)])

# List of pre-defined predicates
Red = Predicate("red", 1)
Red.apply = lambda obj : (obj.color == 0)

Blue = Predicate("blue", 1)
Blue.apply = lambda obj : (obj.color == 1)

Green = Predicate("green", 1)
Green.apply = lambda obj : (obj.color == 2)

Near = Predicate("near", 2)
Near.apply = lambda obj1, obj2: Objects.dist(obj1, obj2) < 0.4

Owns = Predicate("owns", 2)
Owns.apply = lambda agent, obj: obj.ownership[agent.id] > 0.8

IsOwned = Predicate("isOwned", 1)
IsOwned.apply = lambda obj: any(map(lambda o:o>0.8, obj.ownership.iteritems()))

InArea = Predicate("inArea", 2)
InArea.apply = lambda obj, area: Objects.inArea(obj, area)
