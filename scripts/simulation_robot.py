#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from beginner_tutorials.srv import CamToAlgPeople
from math import pow,atan2,sqrt,cos,sin
import math

class Robot :

    def __init__(self) :
        rospy.init_node('robot_controller', anonymous=True)
        self.left_motor_publisher = rospy.Publisher('/leftMotorSpeed', Float32, queue_size=10)
        self.right_motor_publisher = rospy.Publisher('/rightMotorSpeed', Float32, queue_size=10)
        self.pos_x_subscriber = rospy.Subscriber('/robotPosX', Float32, self.pos_x_callback)
        self.pos_y_subscriber = rospy.Subscriber('/robotPosY', Float32, self.pos_y_callback)
        self.angle_subscriber = rospy.Subscriber('/robotAngle', Float32, self.angle_callback)
        self.pos_x = 0
        self.pos_y = 0
        self.angle = 0
        self.goal_x = 0.1
        self.goal_y = 0.1
        self.rate = rospy.Rate(100)

    def get_angle(self,x1, y1, x2, y2) :
        X1 = x1 / sqrt(pow(x1, 2) + pow(y1, 2))
        Y1 = y1 / sqrt(pow(x1, 2) + pow(y1, 2))
        X2 = x2 / sqrt(pow(x2, 2) + pow(y2, 2))
        Y2 = y2 / sqrt(pow(x2, 2) + pow(y2, 2))

        det = X1 * Y2 - Y1 * X2
        dot = X1 * X2 + Y1 * Y2

        return atan2(det, dot)

    def get_distance(self):
        distance = sqrt(pow((self.goal_x - self.pos_x), 2) + pow((self.goal_y - self.pos_y), 2))
        return distance

    def loop(self) :
        self.load_goal_position()
        left_motor_speed = math.pi
        right_motor_speed = math.pi
        delta_angle = self.get_angle(cos(self.angle), sin(self.angle), self.goal_x - self.pos_x, self.goal_y - self.pos_y)
        print(delta_angle)

        if self.get_distance() < 0.1 :
            left_motor_speed = 0
            right_motor_speed = 0
        if delta_angle < 0 :
            print "left"
            right_motor_speed = right_motor_speed / delta_angle / 10
        elif delta_angle > 0 :
            print "right"
            left_motor_speed = left_motor_speed / delta_angle / 10

        self.left_motor_publisher.publish(left_motor_speed)
        self.right_motor_publisher.publish(right_motor_speed)
        self.rate.sleep()

    def load_goal_position(self):
        rospy.wait_for_service('people')
        try:
            people = rospy.ServiceProxy('people', CamToAlgPeople)
            resp =people(1)
            lgt=len(resp.positions);
            if lgt >= 2 :
                self.goal_x = resp.positions[0]
                self.goal_y = resp.positions[1]/4
                print (self.goal_x, self.goal_y)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pos_x_callback(self, data) :
        self.pos_x = data.data

    def pos_y_callback(self, data) :
        self.pos_y = data.data

    def angle_callback(self, data) :
        self.angle = data.data

    def update(self) :
        #self.left_motor_publisher.publish(10)
        #self.right_motor_publisher.publish(5)
        self.loop()
        print(self.pos_x, "; ", self.pos_y, "; ", self.angle)


if __name__ == '__main__':
    try:
        #Testing our function
        robot = Robot()
        while 1:
            robot.update()

    except rospy.ROSInterruptException: pass


"""class turtlebot():

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
        while 1:
            x.move2goal()
            x.loadGoalPosition();

    except rospy.ROSInterruptException: pass
"""
