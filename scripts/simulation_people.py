#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt,cos,sin,pi


def getAngle(x1, y1, x2, y2) :
    X1 = x1 / sqrt(pow(x1, 2) + pow(y1, 2))
    Y1 = y1 / sqrt(pow(x1, 2) + pow(y1, 2))
    X2 = x2 / sqrt(pow(x2, 2) + pow(y2, 2))
    Y2 = y2 / sqrt(pow(x2, 2) + pow(y2, 2))

    det = X1 * Y2 - Y1 * X2
    dot = X1 * X2 + Y1 * Y2

    return atan2(det, dot)

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.posex=0;
        self.posey=0;
        self.posetheta=0;
        self.rate = rospy.Rate(5)
        self.time1 = 0
        self.time2 = 0
        self.updatePos = 0

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.posex), 2) + pow((goal_y - self.posey), 2))
        return distance

    def loop(self, vel_msg, goal_posex, goal_posey, isEnd = False) :

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        lastx = vel_msg.linear.x
        lastz = vel_msg.angular.z

        if isEnd :
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        else :
            if self.updatePos == 0 :
                vel_msg.linear.x = (0.75 * sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)))
                vel_msg.angular.z = 0
            elif self.updatePos == 1 :
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            elif self.updatePos == 2 :
                vel_msg.linear.x = 0
                vel_msg.angular.z = 4*getAngle(cos(self.posetheta), sin(self.posetheta), goal_posex - self.posex, goal_posey - self.posey)
            else :
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        self.updatePos = (self.updatePos + 1) % 4
        self.velocity_publisher.publish(vel_msg)
        self.time2 = rospy.Time.now().to_sec()

        if self.time1 != 0 :
            if (self.updatePos - 2) % 4 == 0 :
                self.posex=self.posex+lastx*cos(self.posetheta)*(self.time2 - self.time1)
                self.posey=self.posey+lastx*sin(self.posetheta)*(self.time2 - self.time1)
            elif (self.updatePos - 2) % 4 == 1 :
                pass
            elif (self.updatePos - 2) % 4 == 2 :
                self.posetheta=(self.posetheta+lastz*(self.time2 - self.time1))%(2*pi)
            else :
                pass

        self.time1 = rospy.Time.now().to_sec()
        self.rate.sleep()


    def move2goal(self):

        goal_posex = input("Set your x goal:")
        goal_posey = input("Set your y goal:")
        distance_tolerance = input("Set your tolerance:")

        vel_msg = Twist()
        lastx = 0
        lastz = 0
        updatePos = False
        time1 = 0

        while self.get_distance(goal_posex, goal_posey) >= distance_tolerance :

            self.loop(vel_msg, goal_posex, goal_posey)

        self.loop(vel_msg, goal_posex, goal_posey, False)
        self.loop(vel_msg, goal_posex, goal_posey, False)




        """ goal_posex = input("Set your x goal:")
        goal_posey = input("Set your y goal:")
        distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        lastx = 0
        lastz = 0
        time=0
        updatePos = True
        delta = 0
        while sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)) >= distance_tolerance:

            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            if updatePos :
                vel_msg.linear.x = (0.75 * sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)));
                vel_msg.angular.z = 0

            else :
                vel_msg.linear.x = 0
                vel_msg.angular.z = getAngle(cos(self.posetheta), sin(self.posetheta), goal_posex - self.posex, goal_posey - self.posey)

            updatePos = not updatePos

            timeold = time
            t1 = rospy.Time.now().to_sec()
            self.velocity_publisher.publish(vel_msg)
            time=rospy.Time.now().to_sec();

            if (timeold!=0):
                if updatePos :
                    self.posex=self.posex+lastx*cos(self.posetheta)*(t1-timeold);
                    self.posey=self.posey+lastx*sin(self.posetheta)*(t1-timeold);
                else :
                    self.posetheta=(self.posetheta+lastz*(t1-timeold))%(2*pi)


            delta = t1-timeold;

            lastx = vel_msg.linear.x
            lastz = vel_msg.angular.z

            self.rate.sleep()


        vel_msg.linear.x = 0
        vel_msg.angular.z =0

        t1 = rospy.Time.now().to_sec()
        self.velocity_publisher.publish(vel_msg)

        if updatePos :
            self.posex=self.posex+lastx*cos(self.posetheta)*(t1-time);
            self.posey=self.posey+lastx*sin(self.posetheta)*(t1-time);
        else :
            self.posetheta=(self.posetheta+lastz*(t1-time))%(2*pi)"""

#        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        while (1):
            x.move2goal()

    except rospy.ROSInterruptException: pass


"""#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt,cos,sin,pi

def getAngle(x1, y1, x2, y2) :
    X1 = x1 / sqrt(pow(x1, 2) + pow(y1, 2))
    Y1 = y1 / sqrt(pow(x1, 2) + pow(y1, 2))
    X2 = x2 / sqrt(pow(x2, 2) + pow(y2, 2))
    Y2 = y2 / sqrt(pow(x2, 2) + pow(y2, 2))

    det = X1 * Y2 - Y1 * X2
    dot = X1 * X2 + Y1 * Y2

    return atan2(det, dot)

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
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        time=0;
        while sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)) >= distance_tolerance:

            lastz=vel_msg.angular.z;

            angle = getAngle(cos(self.posetheta), sin(self.posetheta), goal_posex - self.posex, goal_posey - self.posey)
            vel_msg.angular.z = angle

            if (distance_tolerance >abs(angle)):
                break

            t1=rospy.Time.now().to_sec();
            if (time!=0):
                self.posetheta=(self.posetheta+lastz*(t1-time))%(2*pi)

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
"""

"""
#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt,cos,sin,pi


def getAngle(x1, y1, x2, y2) :
    X1 = x1 / sqrt(pow(x1, 2) + pow(y1, 2))
    Y1 = y1 / sqrt(pow(x1, 2) + pow(y1, 2))
    X2 = x2 / sqrt(pow(x2, 2) + pow(y2, 2))
    Y2 = y2 / sqrt(pow(x2, 2) + pow(y2, 2))

    det = X1 * Y2 - Y1 * X2
    dot = X1 * X2 + Y1 * Y2

    return atan2(det, dot)

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
        updatePos = False
        while sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)) >= distance_tolerance:

            #Porportional Controller
            if updatePos :
                #linear velocity in the x-axis:
                lastx=vel_msg.linear.x;
                vel_msg.linear.x = (0.75 * sqrt(pow((goal_posex - self.posex), 2) + pow((goal_posey - self.posey), 2)));

                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            else :
                #angular velocity in the z-axis:
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                lastz=vel_msg.angular.z;
                #vel_msg.angular.z = 4 * (atan2(goal_posey - self.posey, goal_posex - self.posex) - self.posetheta)
                angle = getAngle(cos(self.posetheta), sin(self.posetheta), goal_posex - self.posex, goal_posey - self.posex)
                vel_msg.angular.z = abs(angle)

            #Publishing our vel_msg

            t1=rospy.Time.now().to_sec();
            if (time!=0):
                self.posex=self.posex+lastx*cos(lastz)*(t1-time);
                print("X : ", self.posex)
                self.posey=self.posey+lastx*sin(lastz)*(t1-time);
                print("Y : ", self.posey)
                self.posetheta=(self.posetheta+lastz*(t1-time))%(2*pi);
                print("Z : ", self.posetheta)
            self.velocity_publisher.publish(vel_msg)
            time=rospy.Time.now().to_sec();
            self.rate.sleep()
            updatePos = not updatePos
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
"""






