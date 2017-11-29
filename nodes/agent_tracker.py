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

        self.lkp_agt_srv = rospy.Service("lookup_agent", LookupAgent,
                                         self.lookupAgentCb)
        self.lst_agt_srv = rospy.Service("list_agents", ListAgents,
                                         self.listAgentsCb)
        self.rst_agt_srv = rospy.Service("reset_agents", Trigger,
                                         self.listAgentsCb)

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
