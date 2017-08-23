import os
import rospy
from objects import *
from ownage_bot.msg import *

class Predicate:
    """Functional representation of object properties and relations."""
    
    def __init__(self, name="", argtypes=[]):
        self.name = name # Human-readable name
        self.n_args = len(argtypes) # Number of arguments
        self.argtypes = argtypes # List of argument types
        self._negated = False # Whether predicate is negated
        self._bindings = [Nil] * self.n_args  # All arguments intially free
        self._apply = lambda *args : True # Implementation of predicate

    def __eq__(self, other):
        """Check for equality of name, argtypes, bindings and negation."""
        if isinstance(other, self.__class__):
            return (self.name == other.name and
                    self.argtypes == other.argtypes and
                    self._bindings == other._bindings and
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
        return tuple(b.toStr() for b in self._bindings)
    
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
        bound._negated = self._negated
        return bound

    def negate(self):
        negation = self.__class__(self.name, self.argtypes)
        negation._bindings = list(self._bindings)
        negation._apply = self._apply
        negation._negated = not self._negated
        return negation
    
    def apply(self, *args):
        """Applies implementation with bindings and negation."""

        # Substitute arguments into Nil slots
        args = list(reversed(args))
        all_args = [args.pop() if a is Nil else a for a in self._bindings]
        if Nil in all_args:
            raise ValueError("Wrong number of arguments.")
        for t, a in zip(self.argtypes, all_args):
            if not isinstance(a, t) and a not is Any:
                raise TypeError("Argument is the wrong type.")

        # No need to evaluate predicate on universe if all are bound
        if Any not in all_args:
            val = self._apply(*all_args)
            return (1.0-val) if self._negated else val

        # Continuously replace all Any arguments
        any_stack = [all_args]
        bound_stack = []
        while len(any_stack > 0):
            cur_args = any_stack.pop()
            if Any not in cur_args:
                bound_stack.append(cur_args)
                continue
            i = cur_args.index(Any)
            for a in self.argtypes[i].universe():
                new_args = list(cur_args)
                new_args[i] = a
                any_stack.append(new_args)

        # Evaluate all substitutions and return maximum
        max_val = 0.0
        for cur_args in bound_stack:
            val = float(self._apply(*all_args))
            max_val = val if val > max_val else max_val
            if max_val == 1.0:
                break
        return (1.0-max_val) if self._negated else max_val

    def toPrint(self):
        """Converts to human-readable string."""
        pre = "not" if self._negated else ""
        pos = " ".join([b.toPrint() for b in self._bindings if b is not None])
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
        # Detect Any or Nil by checking for underscores
        bindings = [Constant.fromStr(s) if (s[0]+s[-1]) == '__'
                    else t.fromStr(s) for t, s in
                    zip(p.argtypes, msg.bindings)]
        p = p.bind(bindings) # This creates a new Predicate object
        p._negated = msg.negated # Which can safely be modified
        return p
    
# List of pre-defined predicates
Red = Predicate("red", [Object])
Red._apply = lambda obj : float(obj.color == "red")

Green = Predicate("green", [Object])
Green._apply = lambda obj : float(obj.color == "green")

Blue = Predicate("blue", [Object])
Blue._apply = lambda obj : float(obj.color == "blue")

Near = Predicate("near", [Object, Object])
Near._apply = lambda obj1, obj2: dist(obj1, obj2) < 0.4

OwnedBy = Predicate("ownedBy", [Object, Agent])
OwnedBy._apply = (lambda obj, agent:
                  0.0 if agent.id not in obj.ownership else
                  obj.ownership[agent.id])

IsOwned = Predicate("isOwned", [Object])
IsOwned._apply = lambda obj: (0.0 if len(obj.ownership) == 0 else
                              max(obj.ownership.values()))

InArea = Predicate("inArea", [Object, Area])
InArea._apply = lambda obj, area: inArea(obj, area)

InCategory = Predicate("inCategory", [Object, Category])
InCategory._apply = lambda obj, c: (0.0 if c not in obj.categories else
                                    obj.categories[c])

IsColored = Predicate("isColored", [Object, Color])
IsColored._apply = lambda obj, col: float(obj.color == col.name)

# List of available predicates for each robotic platform
if os.getenv("OWNAGE_BOT_PLATFORM", "baxter") == "baxter":
    # Only Baxter is currently supported
    db = [Red, Green, Blue, OwnedBy, IsOwned,
          InArea, InCategory, IsColored]
else:
    db = []
db = dict([(p.name, p) for p in db])
