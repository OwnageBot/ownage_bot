#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *

class AgentTracker:
    """Node that tracks and updates agent properties."""

    def __init__(self):
        # Database of known agents
        self.agent_db = dict()
        # Agent currently interacting with the system
        self.current_agent = None

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
        self.rst_agt_srv = rospy.Service("reset_agents", Trigger,
                                         self.resetAgentsCb)

    def agentInputCb(self, msg):
        """Updates database and current agent with new information."""
        self.current_agent = Agent.fromMsg(msg)
        self.agent_db[msg.id] = self.current_agent
        self.new_agent_pub.publish(msg)
        
    def lookupAgentCb(self, req):
        """ Returns properties of requested agent."""
        if req.id in self.agent_db:
            agt = self.agent_db[req.id]
            return LookupAgentResponse(True, agt.toMsg())
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
        

if __name__ == '__main__':
    rospy.init_node('agent_tracker')
    objectTracker = AgentTracker()
    rospy.spin()
