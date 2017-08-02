import predicates
import actions

class Rule:
    """Condition-action pairs that the robot should follow."""    
    
    # Constants defining rule types
    forbidden = "forbidden"
    allowed = "allowed"
    obligatory = "obligatory"
    
    def __init__(self, conditions=[], action=actions.Empty,
                 detype="forbidden"):
        # List of predicate-substitution pairs 
        self.conditions = conditions
         # Action to be performed
        self.action = action
         # Deontic operator type
        self.detype = detype
        
    def evaluate(self, obj):
        """Evaluates if object satisfies the predicates."""
        for p, sub in self.conditions:
            # Replace None with the object in substitution list
            sub = [arg if arg not is None else obj for arg in sub]
            if not p.apply(sub):
                return 0.0
        return 1.0

    def toString(self):
        return " ".join([self.action.name, "on"] +
                        [p.name for p, sub in self.conditions] +
                        ["target","is",self.detype])
        
# List of pre-defined rules
DoNotTouchRed = Rule([(predicates.Red, None)], actions.PickUp)
DoNotTouchGreen = Rule([(predicates.Green, None)], actions.PickUp)
DoNotTouchBlue = Rule([(predicates.Blue, None)], actions.PickUp)
DoNotTouchOwned = Rule([(predicates.IsOwned, None)], actions.PickUp)

DoNotTrashRed = Rule([(predicates.Red, None)], actions.Trash)
DoNotTrashGreen = Rule([(predicates.Green, None)], actions.Trash)
DoNotTrashBlue = Rule([(predicates.Blue, None)], actions.Trash)
DoNotTrashOwned = Rule([(predicates.IsOwned, None)], actions.Trash)
