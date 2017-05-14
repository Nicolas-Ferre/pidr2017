#!/usr/bin/env python

import sys
import rospy
import pylab as pl
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from beginner_tutorials.srv import CamToAlg

xt=[];
yt=[];
c=0;

def algo():
    # Starts a new node
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    x=0;
    y=0;
    angle=0;
    global back;
    global speed;
    vel_msg.linear.z = 0
    vel_msg.angular.y = 0
    vel_msg.angular.x = 0
    vel_msg.angular.z = 0
    rate = rospy.Rate(10)
    i=0;
    while not rospy.is_shutdown():
        rate.sleep()
        i=i+1;
        if i==50:
            i=0
            takepicture(x,y,angle)
            angle=(angle+np.pi/2)%(2*np.pi)
            print("ok")
            

def takepicture(x,y,theta):
    print "ok1"
    rospy.wait_for_service('claques')
    print "ok2"
    try:
        vision=rospy.ServiceProxy('claques', CamToAlg)
        print "ok4"
        data=vision(1);
        print "ok5"
        print data
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "ok3"
    global xt;
    global yt;
    x1=[];
    y1=[];
    lgt=len(data.tranche);
    i=0;
    for fl in data.tranche:
        if fl<7.5:
            xt.append(x+(-fl*np.cos(35/180*np.pi+np.pi*i*110/(lgt*180)+35./180*np.pi + theta)));
            yt.append(y+(fl*np.sin(35/180*np.pi+np.pi*i*110/(lgt*180)+35./180*np.pi + theta)));
        i=i+1;
    pl.plot(xt,yt,'bo');
    pl.draw();
    pl.savefig('./capture/alldone', dpi=300)
    pl.close()

    
def move(vel_msg,i):
    vel_msg.linear.x = i
    
    
def comp():
    rospy.init_node('robot_comportement', anonymous=True)
    algo()
    
if __name__ == '__main__':
    try:
        #Testing our function
        comp()
    except rospy.ROSInterruptException: pass
