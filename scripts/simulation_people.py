#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from beginner_tutorials.srv import CamToAlgPeople
from math import pow,atan2,sqrt,cos,sin



class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)   
        self.pose=Pose();
        self.goalx=0;
        self.goaly=0;
        self.distance_tolerance=0.1;
        self.rate = rospy.Rate(5)


    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)-5.544445
        self.pose.y = round(self.pose.y, 4)

    def move2goal(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        while self.get_distance() >= self.distance_tolerance :
            self.loop(vel_msg);
        vel_msg.linear.x =0;              
        vel_msg.angular.z =0;
        self.velocity_publisher.publish(vel_msg);
  
  
  
    def loop(self, vel_msg) :
        a=self.loadGoalPosition();
        if (a==-1):
            print "error";
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.linear.x = (1.5 * self.get_distance())              
        vel_msg.angular.z = 4*self.getAngle(cos(self.pose.theta), sin(self.pose.theta), self.goalx - self.pose.x, self.goaly - self.pose.y)
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def loadGoalPosition(self):
        print "1"
        rospy.wait_for_service('people')
        print "1"
        try:
            getPeoples = rospy.ServiceProxy('people', CamToAlgPeople)
            resp =getPeoples(1)
            lgt=len(resp.positions);
            if (lgt>=2):
                self.goalx=resp.positions[0];
                self.goaly=resp.positions[1];
                print (self.goalx,self.goaly)
                return;
            else:
                return -1;
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def get_distance(self):
        distance = sqrt(pow((self.goalx - self.pose.x), 2) + pow((self.goaly - self.pose.y), 2))
        return distance
        
        
    def getAngle(self,x1, y1, x2, y2) :
        X1 = x1 / sqrt(pow(x1, 2) + pow(y1, 2))
        Y1 = y1 / sqrt(pow(x1, 2) + pow(y1, 2))
        X2 = x2 / sqrt(pow(x2, 2) + pow(y2, 2))
        Y2 = y2 / sqrt(pow(x2, 2) + pow(y2, 2))
    
        det = X1 * Y2 - Y1 * X2
        dot = X1 * X2 + Y1 * Y2
    
        return atan2(det, dot)
 

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        while (1):
            x.move2goal()
            x.loadGoalPosition();

    except rospy.ROSInterruptException: pass

