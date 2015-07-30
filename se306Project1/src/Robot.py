#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math


class Robot:

    def __init__(self,r_id,x_off,y_off):

        #declaring the instance variables
        self.robot_id = 0
        self.linearX = 0
        self.angularZ = 0
        self.theta = 0
        self.px = x_off
        self.py = y_off
        self.robot_id = r_id
        self.robot_node_name = ("RobotNode" +str(r_id))
        self.robot_node_identifier = ("robot_"+ str(r_id))

        #Node Initiation
        rospy.init_node(self.robot_node_name)

        #setting up publishers and subscribers
        self.RobotNode_stage_pub = rospy.Publisher(self.robot_node_identifier+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        self.StageOdo_sub = rospy.Subscriber(self.robot_node_identifier+"/odom", nav_msgs.msg.Odometry, self.StageOdom_callback)

        self.RobotNode_cmdvel = geometry_msgs.msg.Twist()

    def StageOdom_callback(self,msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.theta = yaw

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)

    def move_forward(self, dist):
        """
        Changes the forward velocity of the robot to 1. It will then move forward, until the distance it has moved forward
        has reached the distance passed to this function.
        """

        #Initiate the distance gained as 0
        dist_gained = 0

        #Iniate a reference to the initial x and y positions
        previousX = self.px
        previousY = self.py

        #While the distance that the robot has gained has not exceeded the given distance, continue to move the robot forward
        while (dist_gained < dist):

            #Calculate remaining distance to travel
            distToGo = dist - dist_gained

            #If the remaining distance is less than 1m, then decelerate the robot. Having a slower moving robot will
            #provide increased accuracy when stopping
            if (distToGo < 1):
                #Set forward velocity to 0.4m/s
                self.RobotNode_cmdvel.linear.x = 0.4
            else:
                #Set forward velocity to 2.0m/s
                self.RobotNode_cmdvel.linear.x = 2


            #Publish the velocity change
            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            #Sleep rospy for 2ms
            rospy.sleep(0.02)

            #Calculate the change in x and y positions from the initial x and y positions, prior to the robot moving
            xDiff = abs(previousX - self.px)
            yDiff = abs(previousY - self.py)

            #Find the distance gained by calculating sqrt(xDiff^2 + yDiff^2)
            dist_gained = math.sqrt(xDiff * xDiff + yDiff * yDiff)

            print("Moving Forward: " + str(distToGo) + "m to go")

        #Stop robot by setting forward velocity to 0 and then publish change
        self.RobotNode_cmdvel.linear.x = 0
        self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

    def turn_left(self):

        pi = math.pi

        thetaTarg = self.theta + math.pi/2

        if (thetaTarg > pi):
            thetaTarg = - pi + (thetaTarg - pi)

        while (abs(self.theta - thetaTarg) > 0.01):
            thetaDiff = abs(self.theta - thetaTarg)

            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = 1
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = 0.1
            else:
                self.RobotNode_cmdvel.angular.z = 0.03

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.01)

            print("Turning left, current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        self.RobotNode_cmdvel.angular.z = 0
        self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

    def turn_right(self):

        pi = math.pi

        thetaTarg = self.theta - math.pi/2

        if (thetaTarg < -pi):
            thetaTarg = pi + (thetaTarg + pi)

        while (abs(self.theta - thetaTarg) > 0.01):
            thetaDiff = abs(self.theta - thetaTarg)

            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = -1
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = -0.1
            else:
                self.RobotNode_cmdvel.angular.z = -0.03

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.01)

            print("Turning left, current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        self.RobotNode_cmdvel.angular.z = 0
        self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)


def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running
    robot0 = Robot(0,5,10)

    rospy.Rate(100)

    rospy.sleep(0.1)

    #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
    #RobotNode_cmdvel = geometry_msgs.msg.Twist()

    while not rospy.is_shutdown():

        robot0.move_forward(5)
        robot0.turn_right()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass