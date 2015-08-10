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

        self.closestRobotID = 0
        self.carrier_robots = ["0,0,0","0,0,0"]
        self.picker_robots = ["0,0,0","0,0,0"]

        self.max_load = 100;
        self.current_load = 0;
        Robot.__init__(self,r_id,x_off,y_off,theta_off)

        # self._actions_ = {
        #     0: self.move_forward,
        #     1: self.goto,
        #     2: self.turn,
        #     3: self.stop,
        #     4: self.gotoClosestRobot,
        # }


        #these variables are used to help the laser callback, it will help in dealing with entities/debris on
        # it's path to the picker robot
        self.StageLaser_sub = rospy.Subscriber(self.robot_node_identifier+"/base_scan",sensor_msgs.msg.LaserScan,self.StageLaser_callback)
        self.ReadLaser = False
        self.FiveCounter = 0
        self._divertingPath_ = False


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


        self.carrier_pub.publish(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        # print("I have sent " + str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta))
        #print("I am at " + xpos + "," + ypos)

        fn = os.path.join(os.path.dirname(__file__), "Carrier"+str(self.robot_id)+".sta")
        output_file = open(fn, "w")
        output_file.write(str(self.robot_node_identifier)+ "\n")
        output_file.write("Carrier\n")
        output_file.write("..........\n")
        output_file.write(str(round(self.px,2)) + "\n")
        output_file.write(str(round(self.py,2)) + "\n")
        output_file.write(str(round(self.theta,2)) + "\n")
        output_file.write(str(self.current_load)+ "/" + str(self.max_load))


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
        #self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2] #+ "," + message.data.split(',')[4]  # Should add element 4 here which is theta
        # print("Carrier array")
        # print ', '.join(self.carrier_robots)
        pass

    """
    @function
    @parameter: message

    Displays info sent from another robot --- used for debugging
    """
    def pickerCallback(self, message):
        # print("Picker callback position " + message.data.split(',')[1] + "," + message.data.split(',')[2])
        self.picker_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2] + "," + message.data.split(',')[4]  # Should add element 3 here which is theta
        print("Picker array")
        print(', '.join(self.picker_robots))


    """
    @function
    @parameter: message

    Determines if a change in path needs to take place when moving from a to b.
    Determines what to do when encountering static and dynamic elements.
    """
    def StageLaser_callback(self, msg):

        #for some reason, ros is passing a value of 5 back every second call regardless if anything
        #is in front of it, this bit of code is just to ignore that random value its passing through

        #print "Header : " + str(msg.header)
        # if not self.ReadLaser:
        #     #print "Not Reading : " + str(msg.ranges[90])
        #     self.ReadLaser = True
        #     return
        # else:
        #     self.ReadLaser = False

        #print "Reading : " + str(msg.ranges[90])
        barCount = 0
        found = False

        #for i in range(0,180):
        #print(msg.ranges[90])
        if msg.ranges[90] < 4.0:
            #action = self._actions_[2], [self, "left"]
            #check if action already exists in stack, otherwise laser will spam rotates
            #if action != self._actionsStack_[-1]:
            #self._stopCurrentAction_ = True
            self.halt_counter += 1
            self._stopCurrentAction_ = True
            self.FiveCounter = 0
            #    self._actionsStack_.append(action)
            #rospy.loginfo("Range at %f degree is: %f", i, msg.ranges[i])
        else:
            self.FiveCounter += 1
            if self.FiveCounter == 5:
                self.halt_counter = 0
                self._stopCurrentAction_ = False
                self.FiveCounter = 0

        if self.halt_counter == 50:
            if not self._divertingPath_:
                print("ENCOUNTERED STATIC ELEMENT!!!!")
                print("Diverting Path Now...")

                move_action = self.move_forward, [10]
                turn_action = self.turn, ["left"]
                move_forward2 = self.move_forward, [5]
                turn_action2 = self.turn, ["right"]

                self._actionsStack_.append(move_action)
                self._actionsStack_.append(turn_action)
                self._actionsStack_.append(move_forward2)
                self._actionsStack_.append(turn_action2)
                self._stopCurrentAction_ = False
            self._divertingPath_ = True
        elif self.halt_counter == 30:
            print("Checking if Entity in front is a static element...")

    """
    @function

    Gets the closest robot out of the array of robots
    """
    def getClosest(self):
        #print("Getting closest robot.....")
        # for index, position in enumerate(self.picker_robots):
        #     current = self.closestRobot
        #     if (self.robot_id != index):
        #         currentDist = self.get_distance(float(current.split(',')[0]), float(current.split(',')[1]))
        #         newDist = self.get_distance(float(position.split(',')[0]), float(position.split(',')[1]))
        #         if (newDist < currentDist):
        #             self.closestRobot = position

        #return robot ID 0
        return 0

    # It is supposed to get the closest robot and then go to that location
    # This doesn't work right as it only calls the getClosest() once and then just continues to call goto()
    # Need a better understanding of how the actions stack works to get this to work correctly
    def waitForPicker(self):
        self.getClosest()
        print("Closest robot is: " + str(self.closestRobotID))
        if(int(self.picker_robots[self.closestRobotID].split(',')[2]) == 20):
            self.goToClosest()

    def goToClosest(self):
        self._stopCurrentAction_ = True
        action = self._actions_[1], [float(self.picker_robots[self.closestRobotID].split(',')[0]), float(self.picker_robots[self.closestRobotID].split(',')[1])]
            #goto(float(self.closestRobot.split(',')[0]), float(self.closestRobot.split(',')[1]))
        if action != self._actionsStack_[-1]:
            #stop moving foward and add turn action
            #self._stopCurrentAction_ = True
            self._actionsStack_.append(action)


