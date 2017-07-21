import rospy
from geometry_msgs import Point
from ownage_bot import Objects
from ownage_bot.msg import ObjectMsg
from ownage_bot.srv import CallAction, CallActionResponse

_service_left = rospy.ServiceProxy(
    "/action_provider/service_left", CallAction)
_service_right = rospy.ServiceProxy(
    "/action_provider/service_right", CallAction)

class Action:
    """Robotic actions performed on objects."""    
    def __init__(self, name="", dependencies=[]):
        self.name = name # Human-readable name
        self.dependencies = [] # List of action dependencies
        self.interface = lambda obj=None : None # Interface with ObjectPicker

    def call(self, target=None):
        """Calls action interface after some checks."""
        return self.interface(target)

# List of pre-defined actions
Scan = Action("scan")
def _scan(target):
    return _service_left("scan", ObjectMsg(), Point())
Scan.interface = _scan

GoHome = Action("goHome")
def _goHome(target):
    return _service_left("home", ObjectMsg(), Point())

PickUp = Action("pickUp")
def _pickUp(target):
    return _service_left("get", target, Point())
PickUp.interface = _pickUp
    
PutDown = Action("putDown")
def _putDown(target):
    return _service_left("put", ObjectMsg(), Point())
PutDown.interface = _putDown

Drop = Action("drop")
def _drop(target):
    return _service_left("drop", ObjectMsg(), Point())
Drop.interface = _drop

Find = Action("find")
def _find(target):
    return _service_left("find", target, Point())
Find.interface = _find

Offer = Action("offer")
def _offer(target):
    return _service_left("offer", target, Point())
Offer.interface = _offer

Trash = Action("trash", [PickUp, Drop])

Collect = Action("collect", [Find, PickUp, GoHome, PutDown])
def _collect(target):
    ret = None
    for a in [Find, PickUp, GoHome, PutDown]:
        ret = a.call(target)
        if not ret.success:
            return ret
    return ret
Collect.interface = _collect

Replace = Action("replace", [PutDown])
def _replace(target):
    return _service_left("replace", ObjectMsg(), Point())
Replace.interface = _replace
