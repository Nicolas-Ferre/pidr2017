#!/usr/bin/env python

import rospy
import pylab as pl
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

x=[];
y=[];
c=0;

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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


def reaction(data):
    global x; 
    global y;
    global c;
    x1=[];
    y1=[];
    lgt=len(data.data);
    i=0;
    for fl in data.data:
        x.append((-fl*np.cos(35/180*np.pi+np.pi*i*110/(lgt*180)+35./180*np.pi)));
        y.append((fl*np.sin(35/180*np.pi+np.pi*i*110/(lgt*180)+35./180*np.pi)));
        i=i+1;
    pl.plot(x,y,'bo');
    pl.draw();
    
    pl.savefig('./capture/alldone', dpi=300)
    pl.close()
    c=c+1;
    
    
def move(vel_msg,i):
    vel_msg.linear.x = i
    
    
def comp():
    rospy.init_node('robot_comportement', anonymous=True)
    rospy.Subscriber('claques', Float32MultiArray, reaction)
    talker()
    
if __name__ == '__main__':
    try:
        #Testing our function
        comp()
    except rospy.ROSInterruptException: pass
