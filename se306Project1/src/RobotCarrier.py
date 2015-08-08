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

The Robot class used to represent a robot in the world stage.
It inherits from the Entity class.

"""
class RobotCarrier(Robot):

    def __init__(self,r_id,x_off,y_off,theta_off):
        self.carrier_pub = rospy.Publisher("carrierPosition",String, queue_size=10)
        self.carrier_sub = rospy.Subscriber("carrierPosition", String, self.carrierCallback)
        self.picker_sub = rospy.Subscriber("pickerPosition", String, self.pickerCallback)

        self.closestRobot = "100,100"
        self.carrier_robots = ["0,0","0,0"]
        self.picker_robots = ["0,0","0,0"]

        self.max_load = 100;
        self.current_load = 0;
        Robot.__init__(self,r_id,x_off,y_off,theta_off)

        self._actions_ = {
            0: self.move_forward,
            1: self.goto,
            2: self.turn,
            3: self.stop,
            4: self.gotoClosestRobot,
        }

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

        self.carrier_pub.publish(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        # print("I have sent " + str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        print("I am at " + xpos + "," + ypos)

        fn = os.path.join(os.path.dirname(__file__), "Carrier"+str(self.robot_id)+"txt")
        output_file = open(fn, "w")
        output_file.write("Name:   "+str(self.robot_node_identifier)+ "\n")
        output_file.write("Type: Carrier\n")
        output_file.write("X Position:   "+ str(self.px) + "\n")
        output_file.write("Y Position:   " +str(self.py) + "\n")
        output_file.write("Theta:   " +str(self.theta))

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)

    """
    @function
    @parameter: message

    Displays info sent from another robot --- used for debugging
    """
    def carrierCallback(self, message):
        # print("Carrier callback position " + message.data.split(',')[1] + "," + message.data.split(',')[2])
        self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2]  # Should add element 3 here which is theta
        # print("Carrier array")
        # print ', '.join(self.carrier_robots)

    """
    @function
    @parameter: message

    Displays info sent from another robot --- used for debugging
    """
    def pickerCallback(self, message):
        # print("Picker callback position " + message.data.split(',')[1] + "," + message.data.split(',')[2])
        self.picker_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2]  # Should add element 3 here which is theta
        print("Picker array")
        print(', '.join(self.picker_robots))


    """
    @function

    Gets the closest robot out of the array of robots
    """
    def getClosest(self):
        print("Getting closest robot.....")
        for index, position in enumerate(self.picker_robots):
            current = self.closestRobot
            if (self.robot_id != index):
                currentDist = self.get_distance(float(current.split(',')[0]), float(current.split(',')[1]))
                newDist = self.get_distance(float(position.split(',')[0]), float(position.split(',')[1]))
                if (newDist < currentDist):
                    self.closestRobot = position

    # It is supposed to get the closest robot and then go to that location
    # This doesn't work right as it only calls the getClosest() once and then just continues to call goto()
    # Need a better understanding of how the actions stack works to get this to work correctly
    def gotoClosestRobot(self):
            self.getClosest()
            print("Closest robot at: " + self.closestRobot)
            while(0 == self.goto(float(self.closestRobot.split(',')[0]), float(self.closestRobot.split(',')[1]))):
                self.getClosest()
