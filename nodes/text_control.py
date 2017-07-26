#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('text_control')
    input_pub = rospy.Publisher("text_input", String, queue_size = 10)
    while not rospy.is_shutdown():
        try:
            data = raw_input('> ')
            input_pub.publish(String(data))
        except EOFError, rospy.ROSInterruptException:
            print "Closing text prompt."
            break

