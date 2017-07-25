from Queue import Queue
import objects
import actions

class Task:
    "Higher-level tasks that construct actions based on the world state."
    
    def __init__(self, name):
        # Human-readable name
        self.name = name
        # Whether task has been completed
        self.done = False
        # Default implementation of updateActions
        self._updateActions = lambda action_queue, object_db : 0;
        # Default implementation of checkActionUndone
        self._checkActionUndone = lambda action, obj : True

    def updateActions(self, action_queue, object_db):
        """Updates the queue of actions to perform based on the object list.
           Returns the number of actions added."""
        if not self.done:
            return self._updateActions(action_queue, object_db)
        return 0
        
    def updateOnce(self, action, obj, action_queue):
        """Prototype action to be used for one-shot tasks."""
        if not self.done:
            action_queue.put((action, obj))
            self.done = True
            return 1
        return 0

    def checkActionUndone(self, action, obj):
        """Double checks if action is still undone."""
        return self._checkActionUndone(action, obj)
    
    @staticmethod
    def oneShot(action, obj=None):
        """Constructs one-shot task from action-object pair."""
        if obj is None:
            name = action.name
        else:
            name = action.name + "Obj" + obj.id
        task = Task(name)
        task._updateActions = lambda action_queue, object_db : \
            task.updateOnce(action, obj, action_queue)
        return task
        
# Pre-defined high-level tasks
Tidy = Task("tidy")
def _tidy(action_queue, object_db):
    return 0
Tidy._updateActions = _tidy
