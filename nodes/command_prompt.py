#!/usr/bin/env python
import rospy
import Queue
from std_msgs.msg import String

out_queue= Queue.Queue()

def outputCb(msg):
    out_queue.put(msg.data)

if __name__ == '__main__':
    rospy.init_node("command_prompt", disable_signals=True)
    input_pub = rospy.Publisher("dialog_in", String, queue_size = 10)
    output_sub = rospy.Subscriber("dialog_out", String, outputCb)

    print "OwnageBot Command Prompt"
    
    while True:
        try:
            data = raw_input('> ')
            input_pub.publish(data)
        except (KeyboardInterrupt, EOFError):
            input_pub.publish("cancel")
            print "Closing text prompt."
            break
        rospy.sleep(0.05)
        while not out_queue.empty():
            print out_queue.get()
