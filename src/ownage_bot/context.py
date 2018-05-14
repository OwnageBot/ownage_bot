"""Helper functions for accessing context variables."""
import rospy
from ownage_bot.msg import *
from ownage_bot.srv import *
from std_srvs.srv import *

_cur_action = rospy.ServiceProxy("cur_action", Trigger)
_cur_target = rospy.ServiceProxy("cur_target", Trigger)
_lookup_agent = rospy.ServiceProxy("lookup_agent", LookupAgent)

def getCurrentAction():
    try:
        return _cur_action().message
    except rospy.ServiceException:
        return None

def getCurrentTarget():
    try:
        return _cur_target().message
    except rospy.ServiceException:
        return None

def getCurrentAgent():
    try:
        return _lookup_agent(-1).agent
    except rospy.ServiceException:
        return None
