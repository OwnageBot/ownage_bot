import rospy
from ownage_bot import Objects
from human_robot_collaboration_msgs.srv import DoAction, DoActionResponse

_service_left = rospy.ServiceProxy(
    "/action_provider/service_left", DoAction)
_service_right = rospy.ServiceProxy(
    "/action_provider/service_right", DoAction)

class Action:
    """Robotic actions performed on objects."""    
    def __init__(self, name="", dependencies=[]):
        self.name = name # Human-readable name
        self.dependencies = [] # List of action dependencies
        self.interface = lambda obj=None : None # Interface with ObjectPicker

    def call(self, obj=None):
        """Calls action interface after some checks."""
        return self.interface(obj)

# List of pre-defined actions
Scan = Action("scan")
def _scan(obj):
    return _service_left("scan", [])
Scan.interface = _scan

GoHome = Action("goHome")
def _goHome(obj):
    return _service_left("home", [])

PickUp = Action("pickUp")
def _pickUp(obj):
    return _service_left("get", [obj.id])
PickUp.interface = _pickUp
    
PutDown = Action("putDown")
def _putDown(obj):
    return _service_left("put", [])
PutDown.interface = _putDown

Drop = Action("drop")
def _drop(obj):
    return _service_left("drop", [])
Drop.interface = _drop

Find = Action("find")
def _find(obj):
    return _service_left("find", [obj.id])
Find.interface = _find

Offer = Action("offer")
def _offer(obj):
    return _service_left("offer", [obj.id])
Offer.interface = _offer

Trash = Action("trash", [PickUp, Drop])

Collect = Action("collect", [Find, PickUp, GoHome, PutDown])
def _collect(obj):
    ret = None
    for a in [Find, PickUp, GoHome, PutDown]:
        ret = a.call(obj)
        if not ret.success:
            return ret
    return ret
Collect.interface = _collect

Replace = Action("replace", [PutDown])
def _replace(obj):
    return _service_left("replace", [])
Replace.interface = _replace
