import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt,cos,sin,pi

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.posex=0;
        self.posey=0;
        self.posetheta=0;
        self.rate = rospy.Rate(10)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_posex = input("Set your x goal:")
        goal_posey = input("Set your y goal:")
        distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()
        time=0;
        while sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            lastx=vel_msg.linear.x;
            vel_msg.linear.x = (0.75 * sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)));
                        
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            lastz=vel_msg.angular.z;
            vel_msg.angular.z = 4 * (atan2(goal_posey - self.posey, goal_posex - self.posex) - self.posetheta)

            #Publishing our vel_msg

            t1=rospy.Time.now().to_sec();
            if (time!=0):
                self.posex=self.posex+lastx*cos(lastz)*(t1-time);
                print(self.posex)
                self.posey=self.posey+lastx*sin(lastz)*(t1-time);
                print(self.posey)
                self.posetheta=(self.posetheta+lastz*(t1-time))%(2*pi);
                print(self.posetheta)
            self.velocity_publisher.publish(vel_msg)
            time=rospy.Time.now().to_sec();
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

#        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        while (1):
            x.move2goal()

    except rospy.ROSInterruptException: pass