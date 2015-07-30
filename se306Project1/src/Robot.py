#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math

class Robot:


    def __init__(self,r_id):

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


    def StageOdom_callback(self,msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        theta = yaw

        rospy.loginfo("Current x position: %f" , self.px)
        rospy.loginfo("Current y position: %f", self.py)
        rospy.loginfo("Current theta: %f", self.theta)


def main():

    robot0 = Robot(0)


    #StageLaser_sub = rospy.Subscriber("robot_0/base_scan",sensor_msgs.msg.LaserScan,StageLaser_callback)

    rospy.Rate(100)

    count = 0

    RobotNode_cmdvel = geometry_msgs.msg.Twist()

    while not rospy.is_shutdown():

        while (robot0.theta < math.pi and robot0.theta>=0):
            RobotNode_cmdvel.angular.z = math.pi*40
            robot0.RobotNode_stage_pub.publish(RobotNode_cmdvel)

            rospy.sleep(1)
            ++count

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass