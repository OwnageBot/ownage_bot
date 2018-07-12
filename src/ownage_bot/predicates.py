import os
import rospy
import copy
from ownage_bot.msg import *
from .objects import *

class Predicate(object):
    """Functional representation of object properties and relations."""
    
    def __init__(self, name="", argtypes=[], speech_fmt=""):
        self.name = name # Human-readable name
        self.n_args = len(argtypes) # Number of arguments
        self.argtypes = argtypes # List of argument types
        self.exc_arg = -1 # Postion of arg for which atoms are exclusive
        self.negated = False # Whether predicate is negated
        self.bindings = [Nil] * self.n_args  # All arguments intially free
        self._apply = lambda *args : True # Implementation of predicate
        self.speech_fmt = speech_fmt # Format for speech output

    def __eq__(self, other):
        """Check for equality of name, argtypes, bindings and negation."""
        if isinstance(other, self.__class__):
            return (self.name == other.name and
                    self.argtypes == other.argtypes and
                    self.bindings == other.bindings and
                    self.negated == other.negated)
            return True
        return NotImplemented

    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented
    
    def __hash__(self):
        """Hash using name, bindings, and negation."""
        return hash((self.name, self.negated, self._bindStrs()))

    def _bindStrs(self):
        """Convert bindings to tuple of strings."""
        return tuple(b.toStr() if type(b) is not list else
                     "|" + "|".join([a.toStr() for a in b]) + "|"
                     for b in self.bindings)

    def copy(self):
        """Creates copy of current predicate."""            
        cp = self.__class__(self.name, self.argtypes)
        cp.exc_arg = self.exc_arg
        cp.bindings = list(self.bindings)
        cp._apply = self._apply
        cp.negated = self.negated
        cp.speech_fmt = self.speech_fmt
        return cp
    
    def bind(self, args):
        """Bind arguments to copy of predicate, None leaves arg unbound."""
        if len(args) != self.n_args:
            raise ValueError("Wrong number of arguments.")

        # Type check for each argument, or internal lists of arguments
        for a, t in zip(args, self.argtypes):
            if a == Nil or a == Any:
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

        bound = self.copy()
        # Bind arguments
        for i, a in enumerate(args):
            if type(a) in [list, tuple]:
                # Make sure internal lists are properly ordered
                bound.bindings[i] = sorted(a, key=lambda x : x.toStr())
            else:
                bound.bindings[i] = a                

        return bound

    def negate(self):
        """Returns negated copy of self."""
        negation = self.copy()
        negation.negated = not self.negated
        return negation

    def generalize(self):
        """Returns generalized copy of self."""
        generalized = self.copy()
        for i, a in generalized.bindings:
            if a == Nil:
                continue
            generalized.bindings[i] = Any
        return generalized
    
    def simplify(self):
        """Simplifies disjunct arguments.

        Example (assuming there are only 3 agents):
        ownedBy(Nil, [1,2,3]) -> ownedBy(Nil, Any)

        Example (assuming Red, Green and Blue are the only colors):        
        isColored(Nil, [Red, Green]) -> not isColored(Nil, Blue)
        """
        simple = self.copy()

        for i in range(self.n_args):
            arg = simple.bindings[i]
            argtype = simple.argtypes[i]
            if type(arg) is not list:
                continue
            universe = argtype.universe()
            # Replace with Any if all possibilities are present
            if len(arg) == len(universe) and arg == universe:
                simple.bindings[i] = Any
                continue
            # Replace with negation of shorter list if exclusivity holds
            if self.exc_arg == i and len(arg) > len(universe)/2:
                for a in arg:
                    universe.remove(a)
                if len(universe) == 1:
                    universe = universe[0]
                simple.bindings[i] = universe
                simple.negated = not simple.negated

        return simple
    
    def apply(self, *args):
        """Applies implementation with bindings and negation."""

        # Substitute arguments into Nil slots
        args = list(reversed(args))
        all_args = [args.pop() if a == Nil else a for a in self.bindings]
        if Nil in all_args:
            raise ValueError("Wrong number of arguments.")

        # Flags which argument positions are lists
        multi_arg_pos = []
        
        # Type check before applying
        for i in range(self.n_args):
            argtype = self.argtypes[i]
            if all_args[i] == Any:
                all_args[i] = argtype.universe() # Replace Any with universe
                multi_arg_pos.append(i)
                continue
            if type(all_args[i]) in [list, tuple]:
                if len(all_args[i]) == 0:
                    raise TypeError("List arguments cannot be empty.")
                for b in all_args[i]:
                    if not isinstance(b, self.argtypes[i]):
                        raise TypeError("Wrong argument type.")
                multi_arg_pos.append(i)
                continue
            if not isinstance(all_args[i], self.argtypes[i]):
                raise TypeError("Argument is the wrong type.")

        # Return early if there are no list arguments
        if len(multi_arg_pos) == 0:
            val = self._apply(*all_args)
            return (1.0-val) if self.negated else val

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
        return neg_val if self.negated else 1-neg_val

    def query(self):
        """Return entities which make predicate true."""
        # TODO: Currently this only works to query one argument
        for b, t in zip(self.bindings, self.argtypes):
            if b != Nil:
                continue
            args = t.universe()
            vals = [self.apply(a) for a in args]
            args_vals = zip(args, vals)
            args_vals.sort(key=lambda x : x[1], reverse=True)
            return args_vals
        return []

    def toPrint(self):
        """Converts to human-readable string."""
        pre = "not" if self.negated else ""
        pos = " ".join([b.toPrint() if type(b) is not list else
                        " or ".join([a.toPrint() for a in b])
                        for b in self.bindings if b != Nil])
        return " ".join([pre, self.name, pos]).strip()

    def toSpeech(self):
        """Converts to string for speech synthesis."""
        speech_fmt = self.speech_fmt
        if speech_fmt == "":
            speech_fmt = "{0} is {n}" + self.name + " {1}" 
        negate_str = "not " if self.negated else ""
        speech_bindings = []
        for b, t in zip(self.bindings, self.argtypes):
            if b == Nil:
                speech_bindings.append(t.nil_str)
            elif b == Any:
                speech_bindings.append(t.any_str)
            elif type(b) is list:
                speech_bindings += [a.toSpeech() for a in b]
            else:
                speech_bindings.append(b.toSpeech())
        return self.speech_fmt.format(*speech_bindings, n=negate_str)
    
    def toMsg(self):
        """Convert to PredicateMsg."""
        msg = PredicateMsg(predicate=self.name, negated=self.negated)
        msg.bindings = self._bindStrs()
        msg.truth = -1.0 if Nil in self.bindings else self.apply()
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
        p.negated = msg.negated # Which can safely be modified
        return p
    
# List of pre-defined predicates
Near = Predicate("near", [Object, Object], "{0} is {n}near to {1}")
Near._apply = lambda o1, o2: dist(o1.position, obj2.position) < 0.4

OwnedBy = Predicate("ownedBy", [Object, Agent], "{0} is {n}owned by {1}")
OwnedBy._apply = (lambda obj, agent: obj.getOwnership(agent.id))

InArea = Predicate("inArea", [Object, Area], "{0} is {n}in {1} area")
InArea._apply = lambda obj, area: inArea(obj, area)

InCategory = Predicate("inCategory", [Object, Category], "{0} is {n}{1}")
InCategory._apply = lambda obj, c: (0.0 if c not in obj.categories else
                                    obj.categories[c])

IsColored = Predicate("isColored", [Object, Color], "{0} is {n}colored {1}")
IsColored._apply = lambda obj, col: float(obj.color == col.name)
IsColored.exc_arg = 1 # Colors are exclusive categories

# List of available predicates for each robotic platform
if os.getenv("OWNAGE_BOT_PLATFORM", "baxter") == "baxter":
    # Only Baxter is currently supported
    db = [OwnedBy, InArea, InCategory, IsColored]
else:
    db = []
db = dict([(p.name, p) for p in db])
