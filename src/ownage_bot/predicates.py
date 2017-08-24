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
        return tuple(b.toStr() if type(b) is not list else
                     "|" + "|".join([a.toStr() for a in b]) + "|"
                     for b in self._bindings)
    
    def bind(self, args):
        """Bind arguments to copy of predicate, None leavs arg unbound."""
        if len(args) != self.n_args:
            raise ValueError("Wrong number of arguments.")

        # Type check for each argument, or internal lists of arguments
        for a, t in zip(args, self.argtypes):
            if a is Nil or a is Any:
                continue
            if type(a) in [list, tuple]:
                if len(a) == 0:
                    raise TypeError("List arguments cannot be empty.")
                for b in a:
                    if not isinstance(b, t):
                        raise TypeError("Wrong argument type.")
                continue
            if not isinstance(a, t):
                raise TypeError("Wrong argument type.")

        bound = self.__class__(self.name, self.argtypes)
        bound._bindings = list(args)
        bound._apply = self._apply
        bound._negated = self._negated

        # Make sure internal lists are properly ordered
        for i, a in enumerate(bound._bindings):
            if type(a) in [list, tuple]:
                bound._bindings[i] = sorted(a, key=lambda x : x.toStr())

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

        # Flags which argument positions are lists
        multi_arg_pos = []
        
        # Type check before applying
        for i, t, a in zip(range(self.n_args), self.argtypes, list(all_args)):
            if a is Any:
                all_args[i] = t.universe() # Replace Any with universe
                multi_arg_pos.append(i)
                continue
            if type(a) in [list, tuple]:
                if len(a) == 0:
                    raise TypeError("List arguments cannot be empty.")
                for b in a:
                    if not isinstance(b, t):
                        raise TypeError("Wrong argument type.")
                multi_arg_pos.append(i)
                continue
            if not isinstance(a, t):
                raise TypeError("Argument is the wrong type.")

        # Return early if there are no list arguments
        if len(multi_arg_pos) == 0:
            val = self._apply(*all_args)
            return (1.0-val) if self._negated else val

        # Expand all internal lists
        cur_stack, new_stack = [all_args], []
        for i in multi_arg_pos:
            for cur_args in cur_stack:
                list_arg = cur_args[i]
                for a in list_arg:
                    new_args = list(cur_args)
                    new_args[i] = a
                    new_stack.append(new_args)
            cur_stack = new_stack
            new_stack = []
                
        # Evaluate all substitutions, combine using noisy or
        neg_val = 1.0
        for cur_args in cur_stack:
            neg_val *= 1-self._apply(*cur_args)
            if neg_val == 0:
                break
        return neg_val if self._negated else 1-neg_val

    def toPrint(self):
        """Converts to human-readable string."""
        pre = "not" if self._negated else ""
        pos = " ".join([b.toPrint() if type(b) is not list else
                        " or ".join([a.toPrint() for a in b])
                        for b in self._bindings if b is not Nil])
        return " ".join([pre, self.name, pos]).strip()
    
    def toMsg(self):
        """Convert to PredicateMsg."""
        msg = PredicateMsg(predicate=self.name, negated=self._negated)
        msg.bindings = self._bindStrs()
        msg.truth = -1.0 if Nil in self._bindings else self.apply()
        return msg
    
    @classmethod
    def fromMsg(cls, msg):
        """Convert from message by looking up database."""
        p = db[msg.predicate] # Base predicate properties should be unmodified
        bindings = []
        for s, t in zip(msg.bindings, p.argtypes):
            if (s[0]+s[-1]) == '__':
                # Detect Any or Nil by checking for underscores
                bindings.append(Constant.fromStr(s))
            elif (s[0]+s[-1]) == '||':
                # Detect internal disjunction by checking for bar '|' symbol
                l = s.strip('|').split('|')
                bindings.append([t.fromStr(a) for a in l])
            else:
                # Directly convert from string to argtype
                bindings.append(t.fromStr(s))
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
