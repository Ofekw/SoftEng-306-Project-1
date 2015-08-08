#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Robot import Robot
import os

"""
@class

The RobotPicker class used to represent a picker in the world stage.
It inherits from the Robot class.

"""
class RobotPicker(Robot):

    def __init__(self,r_id,x_off,y_off,theta_off):
        self.picker_pub = rospy.Publisher("pickerPosition",String, queue_size=10)

        self.max_load = 20;
        self.current_load = 0;
        Robot.__init__(self,r_id,x_off,y_off,theta_off)

    def robot_specific_function(self):
        pass

    """
    @function
    @parameter: Msg msg

    Callback function to update position and other odometry values
    """
    def StageOdom_callback(self,msg):

        #Update the px and py values
        self.update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

        #Find the yaw from the quaternion values
        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        #Update the theta value
        self.update_theta(yaw)

        xpos = str(self.px)
        ypos = str(self.py)
        #com_pub.publish("\n" + rospy.get_caller_id() +  " is at position x: " + xpos + "\nposition y: " + ypos)

        self.picker_pub.publish(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        print("I have sent " + str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))

        fn = os.path.join(os.path.dirname(__file__), "Picker"+str(self.robot_id)+".txt")
        output_file = open(fn, "w")
        output_file.write("Name:   "+str(self.robot_node_identifier)+ "\n")
        output_file.write("Type: Picker\n")
        output_file.write("X Position:   "+ str(self.px) + "\n")
        output_file.write("Y Position:   " +str(self.py) + "\n")
        output_file.write("Theta:   " +str(self.theta))

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)