#!/usr/bin/env python


import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('claques', String, queue_size=10)
    rospy.init_node('cam', anonymous=True)
    while not rospy.is_shutdown():
        hello_str=input("pichnette :")
        pub.publish(str(hello_str))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
