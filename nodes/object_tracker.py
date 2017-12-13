#!/usr/bin/env python
import rospy
from std_srvs.srv import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *

class ObjectTracker:
    """Base class for tracking and updating object properties."""

    def __init__(self):
        super(ObjectTracker, self).__init__()

        # Database of tentative objects
        self.tentative_db = dict()
        # Database of tracked objects
        self.object_db = dict()

        self.avatar_ids = rospy.get_param("avatar_ids", [])

        # Servers for querying and modifying tracked object data
        self.lkp_obj_srv = rospy.Service("lookup_object", LookupObject,
                                         self.lookupObjectCb)
        self.lst_obj_srv = rospy.Service("list_objects", ListObjects,
                                         self.listObjectsCb)
        self.rst_obj_srv = rospy.Service("reset_objects", Trigger,
                                          self.resetObjectsCb)

        # Publishers, subscribers, etc
        self.new_obj_pub = rospy.Publisher("new_object",
                                           ObjectMsg, queue_size = 10)
        self.owner_sub = rospy.Subscriber("owner_prediction",
                                          ObjectMsg, self.ownershipCb)
                        
    def ownershipCb(self, msg):
        """Callback for owner prediction, updates database accordingly."""
        obj = Object.fromMsg(msg)
        self.object_db[obj.id].ownership = dict(obj.ownership)
                
    def lookupObjectCb(self, req):
        """ Returns properties of particular object"""
        if req.id in self.object_db:
            obj = self.object_db[req.id]
            return LookupObjectResponse(True, obj.toMsg())
        else:
            return LookupObjectResponse(False, ObjectMsg())

    def listObjectsCb(self, req):
        """Returns list of tracked objects"""
        return ListObjectsResponse([obj.toMsg() for
                                    obj in self.object_db.values()])

    def resetObjectsCb(self, req):
        """Clears the object databases."""
        self.tentative_db.clear()
        self.object_db.clear()
        return TriggerResponse(True, "")
    
if __name__ == '__main__':
    rospy.init_node('object_tracker')
    ObjectTracker()
    rospy.spin()
