#!/usr/bin/env python
import rospy
from aruco_tracker import ArUcoTracker
from endpoint_tracker import EndpointTracker

class BaxterTracker(ArUcoTracker, EndpointTracker):
    """Combines ArUco tracking and endpoint tracking into one node."""

    def __init__(self):
        super(BaxterTracker, self).__init__()
    
if __name__ == '__main__':
    rospy.init_node('object_tracker')
    BaxterTracker()
    rospy.spin()
