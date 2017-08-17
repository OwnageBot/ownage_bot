#!/usr/bin/env python
import math
import random as r
import threading
from copy import copy
import rospy
import ownage_bot
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

class WorldSimulator():
    """Generates and maintains a simulated world of objects and avatars."""
    
    def __init__(self):
        """Sets up servers and databases."""
        # Database of scenario generation functions
        self.scenario_db = dict()
        
        # Databases of simulated objects and agents
        self.object_db = dict()
        self.agent_db = dict()

        # Lock that guards changes to the world model
        self.lock = threading.Lock()
        
        # Robot location parameters
        self.ground_lvl = 0.0
        self.arm_lvl = 0.30
        self.home_pos = Point(*rospy.get_param("~home_pos", [0, 0, 0.30]))
        
        # Object and avatar base IDs
        self.obj_base_id = rospy.getparam("~obj_base_id", 1)
        self.avt_base_id = rospy.getparam("~avt_base_id", 100)        
        
        # Which object is currently gripped, -1 if none
        self.gripped = {"left": -1, "right": -1}
        # Last pick up location for each arm
        self.pick_loc = {"left": Point(), "right": Point()}
        
        # Servers that list objects and respond to actions
        self.lkp_obj_srv = \
            rospy.Service("lookup_object", LookupObject, self.lookupObjectCb)
        self.lst_obj_srv = \
            rospy.Service("list_objects", ListObjects, self.listObjectsCb)
        self.lkp_agt_srv = \
            rospy.Service("lookup_object", LookupAgent, self.lookupAgentCb)
        self.lst_agt_srv = \
            rospy.Service("list_objects", ListAgents, self.listAgentsCb)
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
        self.lock.acquire()
        if req.id in self.object_db:
            obj = self.object_db[req.id]
            self.lock.release()
            return LookupObjectResponse(True, obj.toMsg())
        else:            
            self.lock.release()
            return LookupObjectResponse(False, ObjectMsg())

    def listObjectsCb(self, req):
        """Returns list of all objects in the world"""
        self.lock.acquire()
        obj_msgs = [obj.toMsg() for obj in self.object_db.values()]
        self.lock.release()
        return ListObjectsResponse(obj_msgs)

    def lookupAgentCb(self, req):
        """ Returns properties of requested agent."""
        self.lock.acquire()
        if req.id in self.agent_db:
            agt = self.agent_db[req.id]
            self.lock.release()
            return LookupAgentResponse(True, agt.toMsg())
        else:            
            self.lock.release()
            return LookupAgentResponse(False, AgentMsg())

    def listAgentsCb(self, req):
        """Returns list of all agents."""
        self.lock.acquire()
        agt_msgs = [agt.toMsg() for agt in self.agent_db.values()]
        self.lock.release()
        return ListAgentsResponse(agt_msgs)
    
    def actionCb(self, arm, req):
        """Updates the world based on requested action."""
        self.lock.acquire()

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

        self.lock.release()
        return resp

    def addScenario(self, name, gen_f):
        """Adds scenario to database."""
        self.scenario_db[name] = gen_f

    def genScenario(self, scenario):
        """Generates specific scenario in database."""
        gen_f = self.scenario_db[scenario]
        self.lock.acquire()
        self.object_db.clear()
        self.agent_db.clear()
        gen_f()
        self.lock.release()

    def genBlocksWorld(self):
        """Generates a world of exclusive-owned colored blocks."""
        n_agents = rospy.get_param("blocks_world/n_agents", 3)
        n_objects = rospy.get_param("blocks_world/n_objects", 20)

        # Variables to cluster blocks by
        cluster_vars = rospy.get_param("blocks_world/cluster_vars",
                                       ['color', 'position', 'proximity'])

        # Distance paramaters
        avatar_rng = rospy.get_param("blocks_world/avatar_rng", [0.4, 0.8])
        cluster_rng = rospy.get_param("blocks_world/cluster_rng", [0.4, 0.8])
        cluster_rad = rospy.get_param("blocks_world/cluster_rad", 0.3)
        object_rng = rospy.get_param("blocks_world/object_rng", [-1.0, 1.0])

        # Update color database
        for c_id in range(n_agents + 1):
            c_name = "color" + str(c_id)
            rospy.set_param_raw("colors/" + c_name, [[0,0,0],[0,0,0]])
        
        # Generate agents and their avatars
        for i in range(n_agents):
            av = Object(id=self.avt_base_id+i, is_avatar=True)
            ag = Agent(id=i+1, avatar_id=av.id)

            dist = r.uniform(*avatar_rng)
            angle = r.uniform(i, i+1) * 2 * math.pi / n_agents
            
            av.position = Point(x=dist*math.cos(angle) + self.home_pos.x,
                                y=dist*math.sin(angle) + self.home_pos.y,
                                z=self.ground_lvl)

            self.agent_db[ag.id] = ag
            self.object_db[av.id] = av

        # Generate cluster centers
        if 'proximity' in cluster_vars:
            # Centers are avatar locations if clustering by proximity
            centers = ([self.home_pos] +
                       [av.position for av in self.object_db.values()])
        else:
            # Otherwise, just cluster based on absolute position
            centers = []
            for i in range(n_agents + 1):
                dist = r.uniform(*cluster_rng)
                angle = r.uniform(i, i+1) * 2 * math.pi / (n_agents + 1)
                
                c = Point(x=dist*math.cos(angle) + self.home_pos.x,
                          y=dist*math.sin(angle) + self.home_pos.y,
                          z=self.ground_lvl)
                centers.append(c)

        # Generate objects within each cluster
        oids = range(obj_base_id, obj_base_id + n_objects)
        for i in range(n_objects):
            obj = Object(id=self.obj_base_id+i)
            c_id = i % (n_agents + 1) # Cluster ID, with 0 unowned
            
            # Generate object locations
            if ('position' in cluster_vars or 'proximity' in cluster_vars):
                # Cluster around centers
                r_offset = r.uniform(*cluster_rad)
                r_angle = r.uniform(0, 2*math.pi)
                c = centers[c_id]
                obj.position = Point(x=c.x + r_offset * math.cos(r_angle),
                                     y=c.y + r_offset * math.sin(r_angle),
                                     z=c.z)
            else:
                # Uniform distribution if no position or proximity clustering
                obj.position = Point(x=r.uniform(*object_rng),
                                     y=r.uniform(*object_rng),
                                     z=self.ground_lvl)
                
            # Generate object colors
            if 'color' in cluster_vars:
                # Associate cluster with color 
                obj.color = "color" + str(c_id)
            else:
                # Choose color uniformly at random
                obj.color = "color" + str(r.choice(range(len(n_agents) + 1)))

            # Assign ownership if not unowned
            if c_id > 0:
                obj.ownership[c_id] = 1.0

            self.object_db[obj.id] = obj
            
    def genSharedDesk(self):
        """Generates a shared office deskspace."""
        pass

    def genAssemblyLine(self):
        """Generates an assembly line with tools and parts."""
        pass
    
if __name__ == '__main__':
    rospy.init_node('world_simulator')
    world_simulator = WorldSimulator()
    scenario = rospy.get_param("~scenario", "blocks_world")
    world_simulator.genScenario(scenario)
    rospy.spin()                                 
