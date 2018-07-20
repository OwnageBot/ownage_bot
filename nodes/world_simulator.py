#!/usr/bin/env python
import math
import random as r
import numpy as np
import threading
from copy import copy
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from ownage_bot import *
from ownage_bot.msg import *
from ownage_bot.srv import *
from geometry_msgs.msg import Point

class WorldSimulator(object):
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
        self.ground_lvl = rospy.get_param("~ground_lvl", 0.0)
        self.arm_lvl = rospy.get_param("~arm_lvl", 0.3)
        self.home_pos = Point(*rospy.get_param("areas/home/center",
                                               [0, 0, 0.30]))
        
        # Object and avatar base IDs
        self.obj_base_id = rospy.get_param("~obj_base_id", 1)
        self.avt_base_id = rospy.get_param("~avt_base_id", 100)        
        
        # Which object is currently gripped, -1 if none
        self.gripped = {"left": -1, "right": -1}
        # Last pick up location for each arm
        self.pick_loc = {"left": Point(), "right": Point()}

        # Server that handles simulation reset
        self.rst_world_srv = rospy.Service("simulation/reset",
                                           Trigger, self.resetCb)
        
        # Servers that list objects and agents
        self.all_obj_srv =  rospy.Service("simulation/all_objects",
                                          ListObjects, self.allObjectsCb)
        self.vis_obj_srv =  rospy.Service("simulation/visible_objects",
                                          ListObjects, self.visibleObjectsCb)
        self.all_agt_srv =  rospy.Service("simulation/all_agents",
                                          ListAgents, self.allAgentsCb)

        # Servers that respond to actions
        self.act_l_srv = \
            rospy.Service("/action_provider/service_left", CallAction,
                          lambda req : self.actionCb("left", req))
        self.act_r_srv = \
            rospy.Service("/action_provider/service_right", CallAction,
                          lambda req : self.actionCb("right", req))
        self.cnc_l_srv = \
            rospy.Service("/action_provider/cancel_left", Trigger,
                          lambda req : TriggerResponse(True, ""))
        self.cnc_r_srv = \
            rospy.Service("/action_provider/cancel_right", Trigger,
                          lambda req : TriggerResponse(True, ""))
        
        # Add some default scenarios
        self.addScenario("blocks_world", self.genBlocksWorld)
        self.addScenario("shared_desk", self.genSharedDesk)
        self.addScenario("assembly_line", self.genAssemblyLine)

    def resetCb(self, req):
        """Clears and regenerates the simulated world."""
        scenario = rospy.get_param("~scenario", "blocks_world")
        self.genScenario(scenario)
        return TriggerResponse(True, "")
        
    def allObjectsCb(self, req):
        """Returns list of all objects in the world."""
        self.lock.acquire()
        obj_msgs = [obj.toMsg() for obj in self.object_db.values()]
        self.lock.release()
        return ListObjectsResponse(obj_msgs)

    def visibleObjectsCb(self, req):
        """Returns list of visible objects."""
        self.lock.acquire()
        obj_msgs = [obj.toMsg() for obj in self.object_db.values()]
        self.lock.release()
        return ListObjectsResponse(obj_msgs)

    def allAgentsCb(self, req):
        """Returns list of all simulated agents."""
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
        gripped_obj = self.object_db.get(gripped_id, None)

        if req.action == CallActionRequest._ACTION_GOHOME:
            if gripped_id >= 0:
                gripped_obj.position = copy(self.home_pos)
        elif req.action == CallActionRequest._ACTION_RELEASE:
            if gripped_id >= 0:
                gripped_obj.position.z = self.ground_lvl
                self.gripped[arm] = -1
        elif req.action == CallActionRequest._ACTION_MOVETO:
            if gripped_id >= 0:
                gripped_obj.position = copy(req.location)
        elif req.action == CallActionRequest._ACTION_FIND:
            if req_id not in self.object_db:
                resp.success = False
                resp.response = "Object {} does not exist".format(req_id)
        elif req.action == CallActionRequest._ACTION_PICKUP:
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
        elif req.action == CallActionRequest._ACTION_PUTDOWN:
            if gripped_id >= 0:
                gripped_obj.position.z = self.ground_lvl
                self.gripped[arm] = -1
            else:
                resp.success = False
                resp.response = "No object is currently being held"
        elif req.action == CallActionRequest._ACTION_OFFER:
            if gripped_id >= 0:
                gripped_obj.position = req.object.position
                gripped_obj.position.z = self.arm_lvl
        elif req.action == CallActionRequest._ACTION_REPLACE:
            if gripped_id >= 0:
                gripped_obj.position = copy(self.pick_loc[arm])
                self.gripped[arm] = -1
            else:
                resp.success = False
                resp.response = "No object is currently being held"
        elif req.action == CallActionRequest._ACTION_WAIT:
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
        """Generates a world of exclusive-owned colored blocks.

        Creates n_agents+1 clusters of objects, each of which is clustered
        by color, position, proximity, or any combination.

        Agent IDs start at 1, and colors are procedurally named as
        'color0', 'color1', etc.
        """
        n_agents = rospy.get_param("blocks_world/n_agents", 3)
        n_objects = rospy.get_param("blocks_world/n_objects", 20)

        # Variables to cluster blocks by
        cluster_vars = rospy.get_param("blocks_world/cluster_vars",
                                       ['color', 'position',
                                        'proximity', 'time'])

        # Distance paramaters
        avatar_rng = rospy.get_param("blocks_world/avatar_rng", [0.4, 0.8])
        cluster_rng = rospy.get_param("blocks_world/cluster_rng", [0.4, 0.8])
        cluster_rad = rospy.get_param("blocks_world/cluster_rad", 0.3)
        object_rng = rospy.get_param("blocks_world/object_rng", [-1.0, 1.0])

        # Color names
        color_names = rospy.get_param("blocks_world/color_names",
                                      ["none", "red", "green", "blue"])
        
        # Avg interaction periods (how frequently objects are interacted with)
        beta_owned = rospy.get_param("blocks_world/beta_owned", 10.0)
        beta_unowned = rospy.get_param("blocks_world/beta_unowned", 1000.0)
        # Store scenario generation time
        t_now = rospy.Time.now()

        # Update color database
        for c_id in range(n_agents + 1):
            if c_id < len(color_names):
                c_name = color_names[c_id]
            else:
                c_name = "color" + str(c_id)
                color_names.append(c_name)
            rospy.set_param("colors/" + c_name, [[0,0,0],[0,0,0]])
        
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
            centers = ([Point(self.home_pos.x,
                              self.home_pos.y,
                              self.ground_lvl)] +
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
        for i in range(n_objects):
            obj = Object(id=self.obj_base_id+i)
            c_id = i % (n_agents + 1) # Cluster ID, with 0 unowned
            
            # Set membership in block category to 1
            obj.categories["block"] = 1.0

            # Generate object locations
            if ('position' in cluster_vars or 'proximity' in cluster_vars):
                # Cluster around centers
                r_offset = r.uniform(0, cluster_rad)
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
                obj.color = color_names[c_id]
            else:
                # Choose color uniformly at random
                col_id = r.choice(range(n_agents+1))
                obj.color = color_names[col_id]

            # Generate last interaction times according to exp. distribution
            if 'time' in cluster_vars:
                for a_id in range(1, n_agents+1):
                    if a_id == c_id:
                        t_last_action = np.random.exponential(beta_owned)
                    else:
                        t_last_action = np.random.exponential(beta_unowned)
                    t_last_action = rospy.Duration(t_last_action)
                    obj.t_last_actions[a_id] = t_now - t_last_action

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
