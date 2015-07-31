#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
import math


def StageOdom_callback(msg):
    px = 5+msg.pose.pose.position.x
    py = 10+msg.pose.pose.position.y
    rospy.loginfo("Current x position is: %f", px)
    rospy.loginfo("Current y position is: %f",py)

def StageLaser_callback(msg):
    barCount = 0
    found = False

    for i in range(0,180):
        if msg.ranges[i] != 5.0:
            rospy.loginfo("Range at %f degree is: %f", i, msg.ranges[i])




def main():

    theta = math.pi/2.0
    px = 10
    py = 20

    linear_x = 0.2
    angular_z = 0.2

    rospy.init_node("RobotNode0")

    #handler initiation here

    RobotNode_stage_pub = rospy.Publisher("robot_0/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

    StageOdo_sub = rospy.Subscriber("robot_0/odom", nav_msgs.msg.Odometry, StageOdom_callback)

    StageLaser_sub = rospy.Subscriber("robot_0/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    rospy.Rate(10)

    count = 0

    RobotNode_cmdvel = geometry_msgs.msg.Twist()



    while not rospy.is_shutdown():

        RobotNode_cmdvel.linear.x = linear_x
        RobotNode_cmdvel.angular.z = angular_z

        RobotNode_cmdvel.linear.x = 0
        RobotNode_cmdvel.angular.z  = 0
        RobotNode_stage_pub.publish(RobotNode_cmdvel)

        #should be spin Once
        #rospy.spin()





        rospy.sleep(0.01)
        ++count

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
