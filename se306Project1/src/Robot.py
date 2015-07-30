#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math


linearX = 0
angularZ = 0
theta = 0

px = 0
py = 0

def StageOdom_callback(msg):
    global px, py, theta

    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y

    (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    theta = yaw

    rospy.loginfo("Current x position: %f" , px)
    rospy.loginfo("Current y position: %f", py)
    rospy.loginfo("Current theta: %f", theta)

def main():

    rospy.init_node("RobotNode0")


    RobotNode_stage_pub = rospy.Publisher("robot_0/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

    RobotQuaternionPub = rospy.Publisher("robot_0/odom", nav_msgs.msg.Odometry, queue_size=10)

    StageOdo_sub = rospy.Subscriber("robot_0/odom", nav_msgs.msg.Odometry, StageOdom_callback)

    #StageLaser_sub = rospy.Subscriber("robot_0/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    rospy.Rate(100)

    count = 0

    RobotNode_cmdvel = geometry_msgs.msg.Twist()

    RobotQuaternion = nav_msgs.msg.Odometry()

    while not rospy.is_shutdown():

        while (theta < math.pi and theta>=0):
            RobotNode_cmdvel.angular.z = math.pi*40
            RobotNode_stage_pub.publish(RobotNode_cmdvel)

            rospy.sleep(1)
            ++count

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass