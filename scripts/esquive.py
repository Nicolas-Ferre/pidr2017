#!/usr/bin/env python
# virgile.dauge@inria.fr
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

distance=0
t0=0
back=2
def talker():
    # Starts a new node
    global distance
    global back
    global t0
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
         
        rate.sleep()


def reaction(data):
    global distance;
    global t0;
    global back;
    t0 = rospy.Time.now().to_sec()
    if back==2:
        back=0;
        distance=-(10-float(data.data)/2);
    
    
def move(vel_msg,i):
    vel_msg.linear.x = i
    
    
def comp():
    rospy.init_node('robot_espquive', anonymous=True)
    rospy.Subscriber('claques', String, reaction)
    talker()
    
if __name__ == '__main__':
    try:
        #Testing our function
        comp()
    except rospy.ROSInterruptException: pass
