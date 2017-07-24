from Queue im
import objects
import actions

class Task:
    "Higher-level tasks that construct actions based on the world state."
    
    def __init__(self, name):
        # Human-readable name
        self.name = name
        # Whether task has been completed
        self.done = False
        # Updates the list of actions to perform based on the list of objects
        # Returns the number of actions added
        self.updateActions = lambda action_queue, object_db : 0;
        # Double checks if action is still undone
        self.checkAction = lambda action, object : True
    
    def updateOnce(self, action, obj, action_queue):
        "Prototype to be used for one-shot tasks."
        if not self.done:
            action_queue.put((action, obj))
            self.done = True
            return 1
        return 0
    
    @staticmethod
    def oneShot(action, obj=None):
        if obj is None:
            name = action.name
        else:
            name = action.name + "Obj" + obj.id
        task = Task(name)
        task.updateActions = lambda action_queue, object_db : \
            task.updateOnce(action, obj, action_queue)
        return task
        
Tidy = Task("tidy")
def _tidy(target):
    return True
Empty.interface = _empty