#!/usr/bin/env python
import rospy
from ownage_bot.msg import *
from ownage_bot.srv import *
from ownage_bot import *
from ownership_tracker import OwnershipTracker

class SimulatedTracker(OwnershipTracker):
    """Simulates object tracking by periodically querying simulator."""

    def __init__(self):
        super(SimulatedTracker, self).__init__()
        # Frequency of updates
        self.simulated_latency =\
            rospy.Duration(rospy.get_param("~simulated_latency", 0.1))
        
        # Set up service to lookup objects from simulator
        self.getSimulated = rospy.ServiceProxy("simulation/visible_objects",
                                               ListObjects)

        # Periodically lookup simulated objects
        rospy.Timer(self.simulated_latency,
                    lambda evt : self.simulatedUpdate())

    def simulatedUpdate(self):
        """Updates all perceivable properties of simulated objects."""
        try:
            rospy.wait_for_service("simulation/visible_objects", 0.5)
        except rospy.ROSException:
            rospy.logwarn("simulation/visible_objects service not available")
            return
        objs = self.getSimulated().objects
        for msg in objs:
            if msg.id not in self.object_db:
                obj = Object.fromMsg(msg)
                obj.t_last_update = rospy.Time.now()
                obj.ownership = dict() # Blind tracker to ownership data
                self.object_db[obj.id] = obj
                super(SimulatedTracker, self).newObjectCb(obj.id)
                self.new_obj_pub.publish(obj.toMsg())
            else:
                obj = self.object_db[msg.id]
                new = Object.fromMsg(msg)
                obj.t_last_update = rospy.Time.now()
                obj.position = new.position
                obj.orientation = new.orientation
                obj.color = new.color
                obj.categories = dict(new.categories)
                obj.t_last_actions = dict(new.t_last_actions)

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    SimulatedTracker()
    rospy.spin()
