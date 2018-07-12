import os
import rospy
from Queue import Queue
from geometry_msgs.msg import Point
from . import objects
from . import actions
from .objects import Object, Area, Location

class Task(object):
    "Higher-level tasks that construct actions based on the world state."
    
    def __init__(self, name):
        # Human-readable name
        self.name = name
        # Whether task has been completed
        self.finished = False
        # Default implementation of updateActions
        self._updateActions = lambda action_queue, object_db : 0;
        # Default implementation of checkActionDone
        self._checkActionDone = lambda action, tgt : False

    def updateActions(self, action_queue, object_db):
        """Updates the queue of actions to perform based on the object list.
           
        action_queue -- a Queue of Action-target pairs
        object_db -- a dict of (id, Object) pairs

        Returns the number of actions added."""
        if not self.finished:
            return self._updateActions(action_queue, object_db)
        return 0
        
    def updateOnce(self, action, tgt, action_queue):
        """Prototype action to be used for one-shot tasks."""
        if not self.finished:
            action_queue.put((action, tgt))
            self.finished = True
            return 1
        return 0

    def checkActionDone(self, action, tgt):
        """Double checks if action is already done."""
        return self._checkActionDone(action, tgt)
    
    @staticmethod
    def oneShot(action, tgt=None):
        """Constructs one-shot task from action-target pair."""
        if not isinstance(tgt, action.tgtype):
            raise ValueError("Target is the wrong type.")
        if tgt is None:
            name = action.name
        elif isinstance(tgt, Object):
            name = action.name + "Obj" + tgt.toStr()
        elif isinstance(tgt, Location):
            name = action.name + "Loc" + tgt.toStr()
        task = Task(name)
        task._updateActions = lambda action_queue, object_db : \
            task.updateOnce(action, tgt, action_queue)
        task._checkActionDone = lambda action, tgt : False
        return task
        
# Pre-defined high-level tasks
Idle = Task("idle")

CollectAll = Task("collectAll")
def _collectAll(action_queue, object_db=dict()):
    """Collects all objects not in the home area."""
    actions_added = 0
    act_list = list(action_queue.queue)
    home_corners = rospy.get_param("areas/home/corners",
                                   [[0.39,0.07], [0.39,0.29],
                                    [0.62,0.29], [0.39,0.29]])
    # Determine uncollected objects
    uncollected = [oid for oid, obj in object_db.iteritems()
                   if not objects.inArea(obj, Area(home_corners))
                   and not obj.is_avatar]
    # Determine objects queued to be collected
    queued = [o.id for a, o in act_list
              if a.name == actions.Collect.name]
    # Scan for more objects if there are no uncollected ones
    if (len(uncollected) == 0 and len(queued) == 0 and 
        all(a.name != actions.Scan.name for a, o in act_list)):
        action_queue.put((actions.Scan, None))
        actions_added = actions_added + 1
    # Append actions for objects not already in queue
    for oid in uncollected:
        if oid not in queued:
            action_queue.put((actions.Collect, object_db[oid]))
            actions_added = actions_added + 1
    return actions_added
CollectAll._updateActions = _collectAll

def _collectAllCheck(action, obj):
    """Check that object is not already in home area before collecting."""
    home_corners = rospy.get_param("areas/home/corners",
                                   [[0.39,0.07], [0.39,0.29],
                                    [0.62,0.29], [0.62,0.07]])
    # Assume undone if action is not Collect
    if action.name != actions.Collect.name:
        return False
    # Do not collect if object is avatar
    if obj.is_avatar:
        return True
    # Return true if object in home area
    return objects.inArea(obj, Area(home_corners))
CollectAll._checkActionDone = _collectAllCheck

TrashAll = Task("trashAll")
def _trashAll(action_queue, object_db):
    """Trashes all objects not in the trash area."""
    actions_added = 0    
    act_list = list(action_queue.queue)
    trash_corners = rospy.get_param("areas/trash/corners",
                                    [[-0.20,0.70], [-0.20,1.00],
                                     [0.10,1.00], [0.10,0.70]])
    # Determine untrashed objects
    untrashed = [oid for oid, obj in object_db.iteritems()
                 if not objects.inArea(obj, Area(trash_corners))
                 and not obj.is_avatar]
    # Determine objects queued to be trashed
    queued = [o.id for a, o in act_list
              if a.name == actions.Trash.name]
    # Scan for more objects if there are no untrashed ones
    if (len(untrashed) == 0 and len(queued) == 0 and 
        all(a.name != actions.Scan.name for a, o in act_list)):
        action_queue.put((actions.Scan, None))
        actions_added = actions_added + 1
    # Append actions for objects not already in queue
    for oid in untrashed:
        if oid not in queued:
            action_queue.put((actions.Trash, object_db[oid]))
            actions_added = actions_added + 1
    return actions_added
TrashAll._updateActions = _trashAll

def _trashAllCheck(action, obj):
    """Check that object is not already in home area before trashing."""
    trash_corners = rospy.get_param("areas/trash/corners",
                                    [[-0.20,0.70], [-0.20,1.00],
                                     [0.10,1.00], [0.10,0.70]])
    # Assume undone if action is not Trash
    if action.name != actions.Trash.name:
        return False
    # Do not trash if object is avatar
    if obj.is_avatar:
        return True
    # Return true if object in home area
    return objects.inArea(obj, objects.Area(trash_corners))
TrashAll._checkActionDone = _trashAllCheck

# List of available tasks for each robotic platform
if os.getenv("OWNAGE_BOT_PLATFORM", "baxter") == "baxter":
    # Only Baxter is currently supported
    db = [Idle, CollectAll, TrashAll]
else:
    db = []
db = dict([(t.name, t) for t in db])
