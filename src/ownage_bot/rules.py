import rospy
import objects
import predicates
import actions
from ownage_bot.msg import *

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
        # Set of predicates that have to be true for the rule to follow
        self.conditions = set(conditions)
        # Deontic operator type
        self.detype = detype

    def __eq__(self, other):
        """Rules are equal if their conditions, actions and types are."""
        if isinstance(other, self.__class__):
            return (self.action.name == other.action.name and
                    self.conditions == other.conditions and
                    self.detype == other.detype)
        return NotImplemented

    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self == other
        return NotImplemented

    def __hash__(self):
        """Hash using action name, condition hash and detype."""
        return hash((self.action.name, tuple(self.conditions), self.detype))
        
    def evaluate(self, tgt, exclusions=set()):
        """Evaluates if target satisfies the predicates."""
        truth = 1.0
        # Default to true if target is None (quick-fix)
        if tgt == None:
            return truth
        # Iterate through all non-excluded predicates
        for p in self.conditions:
            if p not in exclusions:
                truth *= p.apply(tgt)
        return truth

    @classmethod
    def evaluateAnd(cls, rule_set, tgt):
        """Evaluates probabilistic conjunction of rules."""

        truth = 1.0
        exclusions = set()
        for r in rule_set:
            # Check for complementary predicates
            if any([p.negate() in exclusions for p in r.conditions]):
                return 0.0
            truth *= r.evaluate(tgt, exclusions)
            # Return early if possible
            if truth == 0.0:
                break
            # Do not double-count identical predicates
            exclusions.add(r.conditions)
        return truth

    @classmethod
    def evaluateOr(cls, rule_set, tgt, exclusions=set()):
        """Evaluates probabilistic disjunction of rules."""
        # Assume false if no rules
        if len(rule_set) == 0:
            return 0.0
        
        # Calculate truth probability of the first rule
        rule_set = set(rule_set)
        cur = rule_set.pop()
        truth = cur.evaluate(tgt, exclusions)

        # Bottom out if only one rule
        if len(rule_set) == 0:
            return truth
        
        # Break up complement of first rule into disjoint parts
        # e.g. !(A*B*C) -> (!A|!B|!C) -> (!A + A*!B + A*B*!C)
        conditions = list(cur.conditions)
        p_parts = [] # e.g. [A, A*!B, A*B*!C]
        p_part_probs = [] # e.g. [P(!A), P(A*!B), P(A*B*!C)]
        p_conj_probs = [1.0] # e.g. [1.0, P(A), P(A*B), P(A*B*C)]
        p_remainders = [] # Remainder rule sets for each part
        for i, p in enumerate(conditions):
            p_prob = p.apply(tgt)
            p_parts.append(conditions[0:i-1] + [p.negate()])
            p_part_probs.append((1-p_prob) * p_conj_probs[-1])
            p_conj_probs.append(p_prob * p_conj_probs[-1])
            p_remainders.append(set(rule_set))

        # Remove rules with complementary predicates
        for r in rule_set:
            for i, part in enumerate(p_parts):
                if any([c.negate() in r.conditions for c in part]):
                    p_remainders[i].remove(r)

        # Recursively calculate probability of the remainder
        for prob, part, remainder in zip(p_part_probs, p_parts, p_remainders):
            # Exclude potentially identical predicates (idempotency)
            truth += prob * cls.evaluateOr(remainder, tgt,
                                           exclusions.union(part))
                    
        return truth

    @classmethod
    def difference(cls, r1, r2):
        """Returns logical subtraction of r2 from r1 as a rule set."""
        if r1.action.name != r2.action.name or r1.detype != r2.detype:
            raise TypeError("Actions and deontic types must match.")
        remainder = set()
        for c in r2.conditions:
            # Return first rule if their intersection is empty
            if c.negate() in r1.conditions:
                return set([r1])
            # Skip subtraction if the result is an empty rule
            if c in r1.conditions:
                continue
            # Intersect first rule with negation of each predicate
            new = cls(r1.action, r1.conditions, r1.detype)
            new.conditions.add(c.negate())
            remainder.add(new)
        return remainder

    @classmethod
    def intersect(cls, r1, r2):
        """Returns logical intersection of two rules."""
        if r1.action.name != r2.action.name or r1.detype != r2.detype:
            raise TypeError("Actions and deontic types must match.")
        new = cls(r1.action, r1.conditions, r1.detype)
        for c in r2.conditions:
            if c.negate() in r1.conditions:
                return cls(r1.action, [], r1.detype)
            new.conditions.add(c)
        return new

    @classmethod
    def merge(cls, r1, r2):
        """Merges two rules."""
        if len(args) < 2:
            raise ValueError("Need to compare at least two rules.")
        pass
    
    def toPrint(self):
        """Converts to human-readable string."""
        return " ".join([self.action.name, "on"] +
                        [p.toPrint() for p in self.conditions] +
                        ["target","is",self.detype])

    def toMsg(self):
        """Convert to ROS message."""
        msg = RuleMsg()
        msg.action = self.action.name        
        msg.conditions = tuple(c.toMsg() for c in self.conditions)
        msg.detype = self.detype
        msg.truth = 1.0
        return msg
        
    @classmethod
    def fromMsg(cls, msg):
        """Convert from ROS message."""
        action = actions.db[msg.action]
        conditions = [predicates.Predicate.fromMsg(c) for
                      c in msg.conditions]
        return cls(action, conditions, msg.detype)
    
# List of pre-defined rules
DoNotTouchRed = Rule(actions.PickUp, [predicates.Red])
DoNotTouchGreen = Rule(actions.PickUp, [predicates.Green])
DoNotTouchBlue = Rule(actions.PickUp, [predicates.Blue])
DoNotTouchOwned = Rule(actions.PickUp, [predicates.IsOwned])

DoNotTrashRed = Rule(actions.Trash, [predicates.Red])
DoNotTrashGreen = Rule(actions.Trash, [predicates.Green])
DoNotTrashBlue = Rule(actions.Trash, [predicates.Blue])
DoNotTrashOwned = Rule(actions.Trash, [predicates.IsOwned])
