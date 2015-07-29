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
    pass

def comCallback(message):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)

def main():

    theta = math.pi/2.0
    px = 10
    py = 20

    linear_x = 0.2
    angular_z = 0.2

    rospy.init_node("RobotNode1")

    #handler initiation here

    RobotNode_stage_pub = rospy.Publisher("robot_1/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

    StageOdo_sub = rospy.Subscriber("robot_1/odom", nav_msgs.msg.Odometry, StageOdom_callback)

    StageLaser_sub = rospy.Subscriber("robot_1/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    com_sub = rospy.Subscriber('communicate',String, comCallback)

    rospy.Rate(10)

    count = 0

    RobotNode_cmdvel = geometry_msgs.msg.Twist()



    while not rospy.is_shutdown():

        RobotNode_cmdvel.linear.x = linear_x
        RobotNode_cmdvel.angular.z = angular_z

        RobotNode_cmdvel.linear.x = 10
        RobotNode_cmdvel.angular.z  = 20
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