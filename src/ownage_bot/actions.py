import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from objects import Object
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
    def __init__(self, name="", tgtype=type(None), dependencies=[]):
        self.name = name # Human-readable name
        self.tgtype = tgtype # Target type
        self.dependencies = dependencies # List of action dependencies
        self._call = lambda obj=None : None # Implementation of call()

    def call(self, target=None):
        """Calls action interface after some checks."""
        try:
            r = self._call(target)
            return r
        except rospy.ROSException:
            return CallActionResponse(False, "")

# Pre-defined actions

Empty = Action("empty")
def _empty(target):
    return True
Empty._call = _empty

Cancel = Action("cancel")
def _cancel(target):
    return _cancel_left()
Cancel._call = _cancel

GoHome = Action("goHome")
def _goHome(target):
    return _service_left("home", ObjectMsg(), Point())
GoHome._call = _goHome

MoveTo = Action("moveTo", Point)
def _moveTo(target):
    return _service_left("move", ObjectMsg(), target)
MoveTo._call = _moveTo

Scan = Action("scan")
def _scan(target):
    return _service_left("scan", ObjectMsg(), Point())
Scan._call = _scan

PickUp = Action("pickUp", Object)
def _pickUp(target):
    return _service_left("get", target.asMessage(), Point())
PickUp._call = _pickUp
    
PutDown = Action("putDown")
def _putDown(target):
    return _service_left("put", ObjectMsg(), Point())
PutDown._call = _putDown

Release = Action("release")
def _release(target):
    return _service_left("release", ObjectMsg(), Point())
Release._call = _release

Find = Action("find", Object)
def _find(target):
    return _service_left("find", target.asMessage(), Point())
Find._call = _find

Offer = Action("offer", Object)
def _offer(target):
    return _service_left("offer", target.asMessage(), Point())
Offer._call = _offer

Trash = Action("trash", Object, [Find, PickUp, MoveTo, Release])
def _trash(target):
    trash_loc = rospy.get_param("trash_area/center", [-0.05, 0.85, 0.20])
    trash_loc = Point(*trash_loc)
    ret = None
    for a, t in [(Find, target), (PickUp, target),
                 (MoveTo, trash_loc), (Release, None)]:
        ret = a.call(t)
        if not ret.success:
            return ret
    return ret
Trash._call = _trash

Collect = Action("collect", Object, [Find, PickUp, GoHome, PutDown])
def _collect(target):
    ret = None
    for a in [Find, PickUp, GoHome, PutDown, GoHome]:
        ret = a.call(target)
        if not ret.success:
            return ret
    return ret
Collect._call = _collect

Replace = Action("replace", None, [PutDown])
def _replace(target):
    return _service_left("replace", ObjectMsg(), Point())
Replace._call = _replace

# List of available actions for each robotic platform
if rospy.get_param("platform", "baxter") == "baxter":
    db = [Cancel, GoHome, MoveTo, Scan, PickUp, PutDown,
          Release, Find, Offer, Trash, Collect, Replace]
else:
    db = []
