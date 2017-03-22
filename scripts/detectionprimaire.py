#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

back=2;
speed=0;

def talker():
    # Starts a new node
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    global back;
    global speed;
    vel_msg.linear.z = 0
    vel_msg.angular.y = 0
    vel_msg.angular.x = 0
    vel_msg.angular.z = 0
    i=0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if i<20 and back==0:
            i=i+1
            move(vel_msg,-speed)
        elif i==20 and back==0:
            back=1
        elif back==1 and i>0:
            move(vel_msg,speed)
            i=i-1
        else :
            move(vel_msg,0)
            back=2;

        velocity_publisher.publish(vel_msg)
        rate.sleep()


def reaction(data):
    global back;
    global speed;
    if back==2:
        back=0;
        speed=float(data.data);
    
    
def move(vel_msg,i):
    vel_msg.linear.x = i
    
    
def comp():
    rospy.init_node('robot_comportement', anonymous=True)
    rospy.Subscriber('claques', String, reaction)
    talker()
    
if __name__ == '__main__':
    try:
        #Testing our function
        comp()
    except rospy.ROSInterruptException: pass
