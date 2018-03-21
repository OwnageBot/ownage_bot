#!/usr/bin/env python
import rospy
from std_srvs.srv import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *

class ObjectTracker(object):
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

        # Variables and services to look up and cache current agent
        self.cur_agt_cache = None
        self.t_agt_cache = rospy.Time()
        self.agt_cache_latency = rospy.get_param("~agt_cache_latency", 0.5)
        self.lookupAgent = rospy.ServiceProxy("lookup_agent", LookupAgent)
        
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

    def getCurrentAgent(self):
        """Looks up current agent, with caching."""
        if (rospy.Time.now() - self.t_agt_cache) > self.agt_cache_latency:
            try:
                rospy.wait_for_service("lookup_agent")
                resp = self.lookupAgent(-1)
                if resp.success:
                    self.cur_agt_cache = Agent.fromMsg(resp.agent)
                else:
                    self.cur_agt_cache = None
            except (rosp.ROSException, rospy.ServiceException):
                # Just return cache if service call could not be executed
                rospy.logwarn("Service error, returning cache instead...")
        return self.cur_agt_cache
    
if __name__ == '__main__':
    rospy.init_node('object_tracker')
    ObjectTracker()
    rospy.spin()
