#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math


"""
@class

The Robot class used to represent a robot in the world stage.
Can go forward and turn left or right or by a certain angle.
"""
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

    """
    @function
    @parameter: Msg msg

    Callback function to update position and other odometry values
    """
    def StageOdom_callback(self,msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.theta = yaw

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)

    """
    @function
    @parameter: int dist


    Moves the Robot forward by a certain specified distance
    """
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
                #Set forward velocity to 0.7m/s
                self.RobotNode_cmdvel.linear.x = 0.7
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


    """
    @function

    @parameter:String direction

    Turn function which allows the robot to turn 90 degrees ( a right angle) either left or right.
    """
    def turn(self, direction):

        pi = math.pi


        if (direction == "left"):
            thetaTarg = self.theta + pi/2
            dir = 1
            if (thetaTarg > pi):
                thetaTarg = - pi + (thetaTarg - pi)
        elif (direction == "right"):
            thetaTarg = self.theta - pi/2
            dir = -1
            if (thetaTarg < -pi):
                thetaTarg = pi + (thetaTarg + pi)

        while (abs(self.theta - thetaTarg) > 0.01):
            thetaDiff = abs(self.theta - thetaTarg)

        #Set the angular velocity to optimal values that don't overshoot pi/2
            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = 2.5 * dir
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = 0.4 * dir
            else:
                self.RobotNode_cmdvel.angular.z = 0.04 * dir

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.0001)

            print("Turning " + direction + " current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        self.RobotNode_cmdvel.angular.z = 0
        self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)


    """
    @function

    @parameter:float angle

    Turn function which allows the robot to turn a specified number of degrees either left or right.
    A negative angle would denote a right rotation and vice-versa
    """
    def rotate(self, angle_in_degrees):

        if (angle_in_degrees<0):
            dir = -1
        else:
            dir = 1
        pi=math.pi
        #convert degrees to radians
        angle_in_radians = (math.pi/180) *angle_in_degrees

        thetaTarg = self.theta + angle_in_radians

        if (thetaTarg > pi):
            thetaTarg = - pi + (thetaTarg - pi)
        elif (thetaTarg < -pi):
            thetaTarg = pi + (thetaTarg + pi)

        while (abs(self.theta - thetaTarg) > 0.01):
            thetaDiff = abs(self.theta - thetaTarg)

        #Set the angular velocity to optimal values that don't overshoot pi/2
            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = 2.5  * dir
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = 0.4 * dir
            else:
                self.RobotNode_cmdvel.angular.z = 0.04 * dir

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.0001)

            print("Rotating - current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        self.RobotNode_cmdvel.angular.z = 0
        self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)




"""
@MAIN

Main function that creates robot and sets path
"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running
    robot0 = Robot(0,5,10)

    rospy.Rate(100)

    rospy.sleep(0.1)

    #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
    #RobotNode_cmdvel = geometry_msgs.msg.Twist()

    while not rospy.is_shutdown():

    #A Zig zag motion that emulates the robots going through rows of trees in the orchard
        for i in range(0,5):
            robot0.move_forward(5)
            robot0.turn("left")
            robot0.move_forward(0.5)
            robot0.turn("left")
            robot0.move_forward(5)
            robot0.turn("right")
            robot0.move_forward(0.5)
            robot0.turn("right")


        # robot0.rotate(-180)
        # rospy.sleep(1)
        # robot0.move_forward(5)
        # robot0.rotate(-90)
        # robot0.move_forward(5)




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass