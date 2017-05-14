#!/usr/bin/env python

import sys
import rospy
import pylab as pl
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from beginner_tutorials.srv import CamToAlg


def algo():
    robotposx=0;
    robotposy=0;
    robotposz=0;
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep();
        moveRobot(robotposx,robotposy);
        
        
        


def moveRobot(x,y):
    [x,y]=requestPos();   
    t0 = rospy.Time.now().to_sec()
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    pos_x=0
    speed=2
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.y = 0
    vel_msg.angular.x = 0
    vel_msg.angular.z = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (pos_x<distance-0.1):
            speed=abs(speed)
            vel_msg.linear.x = speed
            velocity_publisher.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            pos_x= pos_x+speed*(t1-t0)
            t0=t1
        elif (pos_x>distance + 0.1):
            speed=-abs(speed)
            vel_msg.linear.x = speed
            velocity_publisher.publish(vel_msg)  
            t1=rospy.Time.now().to_sec()
            pos_x= pos_x+speed*(t1-t0)
            t0=t1
        else:
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            back=2;
         
def requestPos():
    try:
        vision=rospy.ServiceProxy('claques', CamToAlg)
        data=position(1);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
    return [x,y]

def comp():
    rospy.init_node('robot_comportement', anonymous=True)
    algo()
    
if __name__ == '__main__':
    try:
        #Testing our function
        comp()
    except rospy.ROSInterruptException: pass
