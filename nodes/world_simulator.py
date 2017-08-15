#!/usr/bin/env python
import math
import random as r
from copy import copy
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

        # Which object is currently gripped, -1 if none
        self.gripped = {"left": -1, "right": -1}
        
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

        resp = CallActionResponse(success=True)
        req_id = req.object.id
        gripped_id = self.gripped[arm]

        if req.action == "home":
            if gripped_id >= 0:
                self.object_db[gripped_id].position = copy(self.home_pos)
        elif req.action == "release":
            if gripped_id >= 0:
                self.object_db[gripped_id].position.z = self.ground_lvl
                self.gripped[arm] = -1
        elif req.action == "move":
            if gripped_id >= 0:
                self.object_db[gripped_id].position = copy(req.location)
        elif req.action == "find":
            if req_id not in self.object_db::
                resp.success = False
                resp.response = "Object {} does not exist".format(req_id)
        elif req.action == "get":
            if gripped_id >= 0:
                resp.success = False
                resp.response = "An object is already being held"
            elif req_id in self.object_db:
                self.gripped[arm] = req_id
                self.pick_loc[arm] = copy(self.object_db[req_id].position)
                self.object_db[req_id].position.z = self.arm_lvl
            else:
                resp.success = False
                resp.response = "Object {} does not exist".format(req_id)
        elif req.action == "put":
            if gripped_id >= 0:
                self.object_db[gripped_id].position.z = self.ground_lvl
                self.gripped[arm] = -1
            else:
                resp.success = False
                resp.response = "No object is currently being held"
        elif req.action == "offer":
            if gripped_id >= 0:
                self.object_db[gripped_id].position = req.object.position
                self.object_db[gripped_id].position.z = self.arm_lvl
        elif req.action == "replace":
            if gripped_id >= 0:
                self.object_db[gripped_id].position = copy(self.pick_loc[arm])
                self.gripped[arm] = -1
            else:
                resp.success = False
                resp.response = "No object is currently being held"
        elif req.action == "wait":
            pass
        return CallActionResponse()

if __name__ == '__main__':
    rospy.init_node('world_simulator')
    world_simulator = WorldSimulator()
    rospy.spin()                                 
