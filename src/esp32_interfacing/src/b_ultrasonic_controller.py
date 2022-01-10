#!/usr/bin/python
import rospy
from std_msgs import Int32
from geometry_msgs.msg import Twist



def CB_ultrasonic_Values( data):

    global Velocity_publisher
        
    turtlebot_vel=Twist()
    ultrasonic_value=data.data
    if ultrasonic_value < 10 :
            turtlebot_vel.linear.x = 0.3
            print(" Forward Moving")
                
    Velocity_publisher.publish(turtlebot_vel)
        
def TB_Control():
    global Velocity_publisher
        Velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        rospy.init_node('turtleBot_ultrasonic_controller')
        rospy.Subscriber('ultrasonic_Values_Topic',Int32,CB_ultrasonic_Values) 
        rospy.spin()

if __name__ == '__main__':
    
    TB_Control()