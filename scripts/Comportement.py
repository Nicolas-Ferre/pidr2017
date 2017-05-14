import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
        
def move ()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
 
