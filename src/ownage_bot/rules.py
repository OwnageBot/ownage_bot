import objects
import predicates
import actions
from ownage_bot.msg import *
from ownage_bot.srv import LookupObject

_lookupObject = rospy.ServiceProxy("lookup_object", LookupObject)

class Rule:
    """Condition-action pairs that the robot should follow."""    
    
    # Constants defining rule types
    forbidden = "forbidden"
    allowed = "allowed"
    obligatory = "obligatory"
    
    def __init__(self, action=actions.Empty, conditions=[],
                 detype="forbidden"):
        # Action to be performed
        self.action = action
        # List of predicate-substitution pairs 
        self.conditions = conditions
        # Deontic operator type
        self.detype = detype

    def __eq__(self, other):
        """Rules are equal if their conditions, actions and types are."""
        if isinstance(other, self.__class__):
            return self.__hash__() == other.__hash__()
        return NotImplemented

    def __ne__(self, other):
        """Define a non-equality test"""
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash using only names, ids, and points."""
        msg = self.toMsg()
        tup = (msg.predicates, msg.args, msg.action, msg.detype)
        return hash(tup)
        
    def evaluate(self, tgt):
        """Evaluates if target satisfies the predicates."""
        for p, subst in self.conditions:
            # Replace None with the object in substitution list
            sub = [arg if arg not is None else tgt for arg in sub]
            if not p.apply(sub):
                return 0.0
        return 1.0

    def toString(self):
        return " ".join([self.action.name, "on"] +
                        [p.name for p, sub in self.conditions] +
                        ["target","is",self.detype])

    def toMsg(self):
        """Convert to ROS message."""
        msg = RuleMsg()
        msg.action = self.action.name
        predicates, args = zip(*self.conditions)
        msg.predicates = tuple([p.name for p in predicates])
        msg.args = tuple(tuple(Rule.argsToStrings(p, a))
                         for p, a in self.conditions)
        msg.detype = self.detype
        msg.confidence = 1.0
        
    @staticmethod
    def fromMsg(msg):
        """Convert from ROS message."""
        action = actions.db[msg.action]
        rule = Rule([], action, msg.detype)
        for p_name, arg_strs in zip(msg.predicates, msg.args):
            p = predicates.db[p_name]
            subst = Rule.stringsToArgs(p, arg_strs)
            rule.conditions.append((p, subst))
        return rule

    @staticmethod
    def argsToStrings(predicate, args):
        strings = []
        for i, a in enumerate(args):
            if a == None:
                s = ''
            elif predicate.argtypes[i] == objects.Object:
                s = a.id
            elif predicate.argtypes[i] == objects.Agent:
                s = a.id                
            elif predicate.argtypes[i] == objects.Area:
                s = str(a.points)
            strings.append(s)
        return strings

    @staticmethod
    def stringsToArgs(predicate, strings):
        args = []
        for i, s in enumerate(strings):
            if s == '':
                a = None
            elif predicate.argtypes[i] == objects.Object:
                a = objects.Object(_lookupObject(int(s)))
            elif predicate.argtypes[i] == objects.Agent:
                a = objects.Agent(int(s))
            elif predicate.argtypes[i] == objects.Area:
                a = objects.Area(eval(s))
            args.append(arg)
        return args
    
# List of pre-defined rules
DoNotTouchRed = Rule(actions.PickUp, [(predicates.Red, [None])])
DoNotTouchGreen = Rule(actions.PickUp, [(predicates.Green, [None])])
DoNotTouchBlue = Rule(actions.PickUp, [(predicates.Blue, [None])])
DoNotTouchOwned = Rule(actions.PickUp, [(predicates.IsOwned, [None])])

DoNotTrashRed = Rule(actions.Trash, [(predicates.Red, [None])])
DoNotTrashGreen = Rule(actions.Trash, [(predicates.Green, [None])])
DoNotTrashBlue = Rule(actions.Trash, [(predicates.Blue, [None])])
DoNotTrashOwned = Rule(actions.Trash, [(predicates.IsOwned, [None])])
