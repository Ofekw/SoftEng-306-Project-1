#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math


def StageOdom_callback(msg):
    px = msg.pose.pose.orientation.z
    py = 10+msg.pose.pose.position.y
   # rospy.loginfo("Current x position is: %f", px)
    #rospy.loginfo("Current y position is: %f",py)
    (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    print(yaw)

def StageLaser_callback(msg):
    pass

def main():

    theta = math.pi/2.0
    px = 10
    py = 20

    linear_x = 0.2
    angular_z = 0.2

    rospy.init_node("RobotNode0")

    #handler initiation here

    RobotNode_stage_pub = rospy.Publisher("robot_0/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

    RobotQuaternionPub = rospy.Publisher("robot_0/odom", nav_msgs.msg.Odometry, queue_size=10)

    StageOdo_sub = rospy.Subscriber("robot_0/odom", nav_msgs.msg.Odometry, StageOdom_callback)

    StageLaser_sub = rospy.Subscriber("robot_0/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    rospy.Rate(100)

    count = 0

    RobotNode_cmdvel = geometry_msgs.msg.Twist()

    RobotQuaternion = nav_msgs.msg.Odometry()



    while not rospy.is_shutdown():

        ()




        RobotNode_cmdvel.angular.z = 0

        RobotNode_cmdvel.linear.x = 0
        RobotNode_stage_pub.publish(RobotNode_cmdvel)

        RobotQuaternion.pose.pose.orientation.w = 0;

      #  RobotQuaternionPub.publish(RobotQuaternion)

        #should be spin Once
        #rospy.spin()





        rospy.sleep(1)
        ++count

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass