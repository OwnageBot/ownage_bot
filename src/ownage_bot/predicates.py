import objects
from objects import Object, Agent, Area
from ownage_bot.msg import *

class Predicate:
    """Functional representation of object properties and relations."""
    
    def __init__(self, name="", argtypes=[]):
        self.name = name # Human-readable name
        self.n_args = len(argtypes) # Number of arguments
        self.argtypes = argtypes # List of argument types
        self._negated = False # Whether predicate is negated
        self._bindings = [None] * self.n_args  # All arguments intially free
        self._apply = lambda *args : True # Implementation of predicate

    def __eq__(self, other):
        """Check for equality of name, argtypes, bindings and negation."""
        if isinstance(other, self.__class__):
            return (self.name == other.name &&
                    self.argtypes == other.argtypes &&
                    self._bindStrs() == other._bindStrs() &&
                    self._negated == other._negated)
            return True
        return NotImplemented

    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented
    
    def __hash__(self):
        """Hash using name, bindings, and negation."""
        return hash((self.name, self._negated, self._bindStrs()))

    def _bindStrs(self):
        """Convert bindings to tuple of strings."""
        return tuple('' if b is None else b.toStr() for b in self._bindings)
    
    def bind(self, args):
        """Bind arguments to copy of predicate, None leavs arg unbound."""
        if len(args) != self.n_args:
            raise ValueError("Wrong number of arguments.")
        for a, t in zip(args, self.argtypes):
            if not isinstance(a, t) and a is not None:
                raise TypeError("Wrong argument type.")
        bound = self.__class__(self.name, self.argtypes)
        bound._bindings = list(args)
        bound._apply = self._apply
        return bound

    def negate(self):
        negation = self.__class__(self.name, self.argtypes)
        negation._bindings = list(self._bindings)
        negation._apply = self._apply
        negation._negated = not self._negated
        return negation
    
    def apply(self, *args):
        """Applies implementation with bindings and negation."""
        # Substitute arguments into empty slots
        args = list(reversed(args))
        all_args = [args.pop() if a is None else a for a in self._bindings]
        if None in all_args:
            raise ValueError("Wrong number of arguments.")
        for t, a in zip(self.argtypes, all_args):
            if not isinstance(a, t):
                raise TypeError("Argument is the wrong type.")
        val = float(self._apply(*all_args))
        return (1.0-val) if self._negated else val

    def toPrint(self):
        """Converts to human-readable string."""
        pre = "not" if self._negated: else ""
        pos = " ".join([b.toStr() for b in self._bindings if b is not None])
        return " ".join([pre, self.name, pos]).strip()
    
    def toMsg(self):
        """Convert to PredicateMsg."""
        msg = PredicateMsg(predicate=self.name, negated=self._negated)
        msg.bindings = self._bindStrs()
        msg.truth = -1.0 if None in self._bindings else self.apply()
        return msg
    
    @classmethod
    def fromMsg(cls, msg):
        """Convert from message by looking up database."""
        p = db[msg.predicate] # Base predicate properties should be unmodified
        bindings = [None if s == '' else t.fromStr(s)
                   for t, s in zip(p.argtypes, msg.bindings)]
        p = p.bind(bindings) # This creates a new Predicate object
        p._negated = msg.negated # Which can safely be modified
        return p
    
# List of pre-defined predicates
Red = Predicate("red", [Object])
Red._apply = lambda obj : (obj.color == "red")

Green = Predicate("green", [Object])
Green._apply = lambda obj : (obj.color == "green")

Blue = Predicate("blue", [Object])
Blue._apply = lambda obj : (obj.color == "blue")

Near = Predicate("near", [Object, Object])
Near._apply = lambda obj1, obj2: objects.dist(obj1, obj2) < 0.4

OwnedBy = Predicate("ownedBy", [Object, Agent])
OwnedBy._apply = lambda obj, agent: obj.ownership[agent.id]

IsOwned = Predicate("isOwned", [Object])
IsOwned._apply = lambda obj: max(obj.ownership.values())

InArea = Predicate("inArea", [Object, Area])
InArea._apply = lambda obj, area: objects.inArea(obj, area)

OfCategory = Predicate("ofCategory", [Object, Category])
OfCategory._apply = lambda obj, c: obj.categories[c]

IsColored = Predicate("isColored", [Object, Color])
OfCategory._apply = lambda obj, col: obj.color == col

# List of available predicates for each robotic platform
if rospy.get_param("platform", "baxter") == "baxter":
    # Only Baxter is currently supported
    db = [Red, Green, Blue, Near, OwnedBy, IsOwned,
          InArea, OfCategory, IsColored]
else:
    db = []
