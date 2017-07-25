import rospy
from std_srvs.srv import Trigger
from geometry_msgs import Point
from ownage_bot.msg import ObjectMsg
from ownage_bot.srv import CallAction, CallActionResponse

_service_left = rospy.ServiceProxy(
    "/action_provider/service_left", CallAction)
_service_right = rospy.ServiceProxy(
    "/action_provider/service_right", CallAction)
_cancel_left = rospy.ServiceProxy(
    "/action_provider/cancel_left", Trigger)
_cancel_right = rospy.ServiceProxy(
    "/action_provider/cancel_right", Trigger)

class Action:
    """Robotic actions performed on objects."""    
    def __init__(self, name="", dependencies=[]):
        self.name = name # Human-readable name
        self.dependencies = [] # List of action dependencies
        self._call = lambda obj=None : None # Implementation of call()

    def call(self, target=None):
        """Calls action interface after some checks."""
        return self._call(target)

# List of pre-defined actions
Empty = Action("empty")
def _empty(target):
    return True
Empty._call = _empty

Cancel = Action("cancel")
def _cancel(target):
    return _cancel_left()
Cancel._call = _cancel

Scan = Action("scan")
def _scan(target):
    return _service_left("scan", ObjectMsg(), Point())
Scan._call = _scan

GoHome = Action("goHome")
def _goHome(target):
    return _service_left("home", ObjectMsg(), Point())

PickUp = Action("pickUp")
def _pickUp(target):
    return _service_left("get", target, Point())
PickUp._call = _pickUp
    
PutDown = Action("putDown")
def _putDown(target):
    return _service_left("put", ObjectMsg(), Point())
PutDown._call = _putDown

Drop = Action("drop")
def _drop(target):
    return _service_left("drop", ObjectMsg(), Point())
Drop._call = _drop

Find = Action("find")
def _find(target):
    return _service_left("find", target, Point())
Find._call = _find

Offer = Action("offer")
def _offer(target):
    return _service_left("offer", target, Point())
Offer._call = _offer

Trash = Action("trash", [PickUp, Drop])

Collect = Action("collect", [Find, PickUp, GoHome, PutDown])
def _collect(target):
    ret = None
    for a in [Find, PickUp, GoHome, PutDown]:
        ret = a.call(target)
        if not ret.success:
            return ret
    return ret
Collect._call = _collect

Replace = Action("replace", [PutDown])
def _replace(target):
    return _service_left("replace", ObjectMsg(), Point())
Replace._call = _replace
