#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('text_control', disable_signals=True)
    input_pub = rospy.Publisher("text_input", String, queue_size = 10)
    while True:
        try:
            data = raw_input('> ')
            input_pub.publish(String(data))
        except (KeyboardInterrupt, EOFError):
            print "Closing text prompt."
            break

