from ownage_bot import Predicates
from ownage_bot import Actions

class Rule:
    """Condition-action pairs that the robot should follow."""    
    
    # Constants defining rule types
    forbidden = "forbidden"
    allowed = "allowed"
    obligatory = "obligatory"
    
    def __init__(self, conditions=[], action=Actions.Empty, detype="forbidden"):
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
DoNotTouchRed = Rule([Predicates.Red], Actions.PickUp)
DoNotTouchBlue = Rule([Predicates.Blue], Actions.PickUp)
DoNotTouchGreen = Rule([Predicates.Green], Actions.PickUp)
DoNotTouchOwned = Rule([Predicates.IsOwned], Actions.PickUp)

DoNotTrashRed = Rule([Predicates.Red], Actions.Trash)
DoNotTrashBlue = Rule([Predicates.Blue], Actions.Trash)
DoNotTrashGreen = Rule([Predicates.Green], Actions.Trash)
DoNotTrashOwned = Rule([Predicates.IsOwned], Actions.Trash)
