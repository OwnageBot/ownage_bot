import predicates
import actions

class Rule:
    """Condition-action pairs that the robot should follow."""    
    
    # Constants defining rule types
    forbidden = "forbidden"
    allowed = "allowed"
    obligatory = "obligatory"
    
    def __init__(self, conditions=[], action=actions.Empty, detype="forbidden"):
        self.conditions = conditions # List of predicates that have to be true
        self.action = action # Action to be performed
        self.detype = detype # Deontic operator type
        
    def evaluate(self, obj):
        """Evaluates if object satisfies the predicates."""
        for condition in self.conditions:
            if not condition.apply(obj):
                return False
        return True
        
# List of pre-defined rules
DoNotTouchRed = Rule([predicates.Red], actions.PickUp)
DoNotTouchGreen = Rule([predicates.Green], actions.PickUp)
DoNotTouchBlue = Rule([predicates.Blue], actions.PickUp)
DoNotTouchOwned = Rule([predicates.IsOwned], actions.PickUp)

DoNotTrashRed = Rule([predicates.Red], actions.Trash)
DoNotTrashGreen = Rule([predicates.Green], actions.Trash)
DoNotTrashBlue = Rule([predicates.Blue], actions.Trash)
DoNotTrashOwned = Rule([predicates.IsOwned], actions.Trash)
