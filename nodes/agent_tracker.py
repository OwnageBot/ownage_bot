#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *

class AgentTracker(object):
    """Node that tracks and updates agent properties."""

    def __init__(self):
        # Database of known agents
        self.agent_db = dict()
        # Agent currently interacting with the system
        self.cur_agent = None

        # Subscriber to new agent introductions
        self.agent_sub = rospy.Subscriber("agent_input", AgentMsg,
                                         self.agentInputCb)

        # Publisher to notify other nodes about new agent
        self.new_agt_pub = rospy.Publisher("new_agent",
                                           AgentMsg, queue_size = 10)
        
        # Services for looking up and resetting agent database
        self.lkp_agt_srv = rospy.Service("lookup_agent", LookupAgent,
                                         self.lookupAgentCb)
        self.lst_agt_srv = rospy.Service("list_agents", ListAgents,
                                         self.listAgentsCb)
        self.set_agt_srv = rospy.Service("set_agents", SendAgents,
                                         self.setAgentsCb)
        self.rst_agt_srv = rospy.Service("reset_agents", Trigger,
                                         self.resetAgentsCb)

        
    def agentInputCb(self, msg):
        """Updates database and current agent with new information."""
        new_agent = Agent.fromMsg(msg)

        # Check if agent ID is already known
        if new_agent.id in self.agent_db:
            self.cur_agent = self.agent_db[new_agent.id]
            return

        # Check if agent name is already known
        if len(new_agent.name > 0):
            for a in self.agent_db.values():
                if new_agent.name == a.name:
                    self.cur_agent = a
                return

        # Insert new agent into database if name is unrecognized
        if agent.id < 0:
            agent.id = len(self.agent_db) + 1
        self.agent_db[agent.id] = agent
        self.cur_agent = agent
        self.new_agt_pub.publish(agent.toMsg())
        
    def lookupAgentCb(self, req):
        """ Returns properties of requested agent."""
        if req.id == -1 and self.cur_agent is not None:
            # Return current agent if ID is -1
            return LookupAgentResponse(True, self.cur_agent.toMsg())
        elif req.id in self.agent_db:
            # Otherwise try to lookup agent in database
            return LookupAgentResponse(True, self.agent_db[req.id].toMsg())
        else:            
            return LookupAgentResponse(False, AgentMsg())

    def listAgentsCb(self, req):
        """Returns list of all agents."""
        agt_msgs = [agt.toMsg() for agt in self.agent_db.values()]
        return ListAgentsResponse(agt_msgs)

    def resetAgentsCb(self, req):
        """Clears the object databases."""
        self.agent_db.clear()
        return TriggerResponse(True, "")

    def setAgentsCb(self, req):
        """Sets database to the received list of agents."""
        self.agent_db.clear()
        for msg in req.agents:
            agent = Agent.fromMsg(msg)
            self.agent_db[agent.id] = agent
            self.new_agt_pub.publish(agent.toMsg())
        return SendAgentsResponse(True)

if __name__ == '__main__':
    rospy.init_node('agent_tracker')
    objectTracker = AgentTracker()
    rospy.spin()
