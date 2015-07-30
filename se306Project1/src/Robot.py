#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math

class Robot:


    def __init__(self, r_id):

        self.robot_id = 0
        self.linearX = 0
        self.angularZ = 0
        self.theta = 0
        self.px = 0
        self.py = 0

        robot_id = r_id

        robot_node_name = ("RobotNode" +str(r_id))

        robot_node_identifier = ("robot_"+ str(r_id))

        rospy.init_node(robot_node_name)

        self.RobotNode_stage_pub = rospy.Publisher(robot_node_identifier+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

        self.RobotQuaternionPub = rospy.Publisher(robot_node_identifier+"/odom", nav_msgs.msg.Odometry, queue_size=10)

        self.StageOdo_sub = rospy.Subscriber(robot_node_identifier+"/odom", nav_msgs.msg.Odometry, self.StageOdom_callback)

        self.RobotNode_cmdvel = geometry_msgs.msg.Twist()


    def StageOdom_callback(self,msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.theta = yaw

        rospy.loginfo("Current x position: %f" , self.px)
        rospy.loginfo("Current y position: %f", self.py)
        rospy.loginfo("Current theta: %f", self.theta)

    def move_forward(self, dist):
        """
        Changes the forward velocity of the robot to 1. It will then move the given distance value in metres.
        """
        dist_gained = 0
        previousX = self.px
        previousY = self.py

        while (dist_gained < dist):
            distToGo = dist - dist_gained

            if (distToGo < 1):
                self.RobotNode_cmdvel.linear.x = 0.4
            else:
                self.RobotNode_cmdvel.linear.x = 2

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.02)

            xDiff = abs(previousX - self.px)
            yDiff = abs(previousY - self.py)

            dist_gained = math.sqrt(xDiff * xDiff + yDiff * yDiff)

            print("Moving Forward: " + str(distToGo) + "m to go")


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

        thetaTarg = self.theta + math.pi/2

        if (thetaTarg < pi):
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

    robot0 = Robot(0)

    #StageLaser_sub = rospy.Subscriber("robot_0/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    rospy.Rate(100)

    count = 0

    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        robot0.move_forward(5)
        robot0.turn_right()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass