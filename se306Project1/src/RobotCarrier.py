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

"""
@class

The Robot class used to represent a robot in the world stage.
It inherits from the Entity class.

"""
class RobotCarrier(Robot):

    def __init__(self,r_id,x_off,y_off):
        global com_pub
        com_pub = rospy.Publisher("carrierPosition",String, queue_size=10)
        self.carrier_sub = rospy.Subscriber("carrierPosition", String, self.carrierCallback)
        self.picker_sub = rospy.Subscriber("pickerPosition", String, self.pickerCallback)

        self.carrier_robots = ["0,0","0,0"]
        self.picker_robots = ["0,0","0,0"]

        self.max_load = 100;
        self.current_load = 0;
        Robot.__init__(self,r_id,x_off,y_off)

        self._actions_ = {
            0: self.move_forward,
            1: self.goto,
            2: self.turn,
            3: self.stop,
            4: self.gotoRobotDemo,
        }

    def robot_specific_function(self):
        pass

    """
    @function
    @parameter: Msg msg

    Callback function to update position and other odometry values
    """
    def StageOdom_callback(self,msg):

        self.px = self.x_off+msg.pose.pose.position.x
        self.py = self.y_off+msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.theta = yaw

        xpos = str(self.px)
        ypos = str(self.py)
        #com_pub.publish("\n" + rospy.get_caller_id() +  " is at position x: " + xpos + "\nposition y: " + ypos)

        com_pub.publish(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        print(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)

    """
    @function
    @parameter: message

    Displays info sent from another robot --- used for debugging
    """
    def carrierCallback(self, message):
        print("Sending position " + message.data.split(',')[1] + "," + message.data.split(',')[2])
        self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2]  # Should add element 3 here which is theta

    """
    @function
    @parameter: message

    Displays info sent from another robot --- used for debugging
    """
    def pickerCallback(self, message):
        print("Sending position " + message.data.split(',')[1] + "," + message.data.split(',')[2])
        self.picker_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2]  # Should add element 3 here which is theta

    """
    @function

    Fuction which gets the closest position out of an array of robot positions
    """
    def getClosest(self):
        current = "10000,10000"
        for index, position in enumerate(self.other_robots):
            print("index" + str(index) + ") Current closest " + current)
            if (self.robot_id != index):
                currentDist = self.getDist(int(current.split(',')[0]), int(current.split(',')[1]))
                newDist = self.getDist(int(position.split(',')[0]), int(position.split(',')[1]))
                if (newDist < currentDist):
                    current = position
                    print("index" + str(index) + ") new closest at " + current)

    def gotoRobotDemo(self):
        self.goto(self.picker_robots[0].split(',')[0],self.picker_robots[0].split(',')[1])