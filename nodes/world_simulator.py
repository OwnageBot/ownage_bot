#!/usr/bin/env python
import math
import random as r
from copy import copy
import rospy
import ownage_bot
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

OBJECT_BASE_ID = 1
AVATAR_BASE_ID = 100

class WorldSimulator():
    """Generates and maintains a simulated world of objects and avatars."""
    
    def __init__(self):
        """Sets up servers and databases."""
        # Database of scenario generation functions
        self.scenario_db = dict()
        
        # Databases of simulated objects and agents
        self.object_db = dict()
        self.agent_db = dict()

        # Robot location parameters
        self.ground_lvl = 0.0
        self.arm_lvl = 0.30
        self.home_pos = Point(*rospy.get_param("home_area/center",
                                               [0.50, 0.176, 0.20]))

        # Which object is currently gripped, -1 if none
        self.gripped = {"left": -1, "right": -1}
        # Last pick up location for each arm
        self.pick_loc = {"left": Point(), "right": Point()}
        
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

        # Add some default scenarios
        self.addScenario("blocks_world", self.genBlocksWorld)
        self.addScenario("shared_desk", self.genSharedDesk)
        self.addScenario("assembly_line", self.genAssemblyLine)
        
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
        return resp

    def addScenario(self, name, gen_f):
        """Adds scenario to database."""
        self.scenario_db[name] = gen_f

    def genScenario(self, scenario):
        """Generates specific scenario in database."""
        gen_f = self.scenario_db[scenario]
        return gen_f()

    def genBlocksWorld(self):
        """Generates a world of colored blocks."""
        pass

    def genSharedDesk(self):
        """Generates a shared office deskspace."""
        pass

    def genAssemblyLine(self):
        """Generates an assembly line with tools and parts."""
        pass

    
if __name__ == '__main__':
    rospy.init_node('world_simulator')
    world_simulator = WorldSimulator()
    world_simulator.genScenario("shared_desk")
    rospy.spin()                                 
