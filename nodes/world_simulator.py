#!/usr/bin/env python
import math
import random as r
import rospy
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
import ownage_bot
from ownage_bot.msg import *
from ownage_bot.srv import *

OBJECT_BASE_ID = 1
AVATAR_BASE_ID = 100

class WorldSimulator():
    """Generates and maintains a simulated world of objects and avatars."""
    
    def __init__(self, scenario):
        """Generates the environment depending on the scenario."""
        # Databases of simulated objects and agents
        self.object_db = dict()
        self.agent_db = dict()
        
        # Servers that list objects and respond to actions
        self.lkp_obj_srv = \
            rospy.Service("lookup_object", LookupObject, self.lookupObjectCb)
        self.lst_obj_srv = \
            rospy.Service("list_objects", ListObjects, self.listObjectsCb)
        self.act_l_srv = \
            rospy.Service("/action_provider/service_left", CallAction,
                          lambda req : self.actionCb("left", req))
        self.act_r_srv = \
            rospy.Service("/action_provider/service_right", CallAction,
                          lambda req : self.actionCb("right", req))

    def lookupObjectCb(self, req):
        """ Returns properties of particular object"""
        if req.id in self.object_db:
            obj = self.object_db[req.id]
            return LookupObjectResponse(True, obj.toMsg())
        else:
            return LookupObjectResponse(False, ObjectMsg())

    def listObjectsCb(self, req):
        """Returns list of all objects in the world"""
        return ListObjectsResponse([obj.toMsg() for
                                    obj in self.object_db.values()])
        
    def actionCb(self, arm, req):
        """Updates the world based on requested action."""
        return CallActionResponse()

if __name__ == '__main__':
    rospy.init_node('world_simulator')
    world_simulator = WorldSimulator()
    rospy.spin()                                 
