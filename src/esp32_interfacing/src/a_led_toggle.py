#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

def toggler():
    pub = rospy.Publisher('toggle_led', Empty, queue_size=10)
    rospy.init_node('toggling_node', anonymous=True)
    msg = Empty()
    pub.publish(Empty)
    

if __name__ == '__main__':
    try:
        toggler()
    except rospy.ROSInterruptException:
        pass