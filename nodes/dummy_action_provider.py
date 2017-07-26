#!/usr/bin/env python
import rospy
from ownage_bot.srv import CallAction, CallActionResponse

def serviceCb(req):
    return CallActionResponse()

if __name__ == '__main__':
    rospy.init_node('action_provider')
    rospy.Service("/action_provider/service_left", CallAction, serviceCb)
    rospy.Service("/action_provider/service_right", CallAction, serviceCb)
                                 