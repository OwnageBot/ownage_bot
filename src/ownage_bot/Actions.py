from ownage_bot import Objects

class Action:
    """Robotic actions performed on objects."""    
    def __init__(self, name="", dependencies=[]):
        self.name = name # Human-readable name
        self.dependencies = [] # List of action dependencies

    def impl(self, obj):
        """Action implementation."""
        pass

# List of pre-defined actions
PickUp = Action("pickUp")
PutDown = Action("putDown")
Drop = Action("drop")
Trash = Action("trash", [PickUp, Drop])
Collect = Action("collect", [PickUp, PutDown])
Replace = Action("replace", [PutDown])
