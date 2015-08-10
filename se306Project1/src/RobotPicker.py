#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import time
import numpy.testing
from Robot import Robot
import Entity
import os

"""
@class

The RobotPicker class used to represent a picker in the world stage.
It inherits from the Robot class.

"""
class RobotPicker(Robot):

    def enum(**enums):
        return type('Enum', (), enums)

    PickerState = enum(PICKING="Picking Fruit",
                              FINDING="Finding Orchard")

    def __init__(self,r_id,x_off,y_off,theta_off):
        self.picker_pub = rospy.Publisher("pickerPosition",String, queue_size=10)

        self.max_load = 20;
        self.current_load = 0;
        self.timeLastAdded = time.clock()

        # self._actions_ = {
        #     0: self.move_forward,
        #     1: self.goto,
        #     2: self.turn,
        #     3: self.stop,
        #     4: self.gotoClosestRobot,
        #     5: self.wait,
        # }

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

        #publish:- id xpos ypos kiwimunber
        self.picker_pub.publish(str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta) + "," + str(self.current_load))
        print("I have sent " + str(self.robot_id) + "," + xpos + "," + ypos+ "," + str(self.theta) + "," + str(self.current_load))

        fn = os.path.join(os.path.dirname(__file__), "Picker"+str(self.robot_id)+".sta")
        output_file = open(fn, "w")
        output_file.write(str(self.robot_node_identifier)+ "\n")
        output_file.write("Picker\n")
        output_file.write(self.state+"\n")
        output_file.write(str(round(self.px,2)) + "\n")
        output_file.write(str(round(self.py,2)) + "\n")
        output_file.write(str(round(self.theta,2)) + "\n")
        output_file.write(str(self.current_load)+ "/" + str(self.max_load))

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)s

    def addKiwi(self, clockTime):
        print("looking to add " + str(self.max_load) + " " + str(self.current_load))
        if(self.current_load >= self.max_load):
            self.waitForCollection()
        elif(clockTime <= (self.timeLastAdded + 1)):
            self.current_load = self.current_load + 1
            print("kiwi added")

    def waitForCollection(self):
        #while(self.current_load >= self.max_load):
            self._stopCurrentAction_ = True
            action = self._actions_[4],[]
            if action != self._actionsStack_[-1]:
                #stop moving foward and add turn action
                self._stopCurrentAction_ = True
                self._actionsStack_.append(action)

            #self.goto(self.px,self.py)
        #    print("ohdera")
            #make the robot stop moving until collected from
        #pass

    def gotoClosestRobot(self):
        pass

    def StageLaser_callback(self, msg):
        barCount = 0
        found = False
        if not self.disableLaser:
            for i in range(70, 110):
                if msg.ranges[i]< 4.0:
                    action = self._actions_[2], [Entity.Direction.RIGHT]
                    #check if action already exists in stack, otherwise laser will spam rotates
                    if action != self._actionsStack_[-1]:
                        #stop moving foward and add turn action
                        self._stopCurrentAction_ = True
                        self._actionsStack_.append(action)
            #check that all lasers in 0-20 range are not hitting object

            rangeCount = 0
            for i in range(160,180):
                if msg.ranges[i]<5.0:
                    rangeCount += 1
            #check if no tree and are waiting for new tree
            if self.noMoreTrees>15 and self.state == self.PickerState.PICKING:
                self.noMoreTrees = 0
                #stop the robot moving forward
                self._stopCurrentAction_ = True
                turnAction = self._actions_[2], [Entity.Direction.LEFT]
                self._actionsStack_.append(turnAction)
            elif rangeCount == 0:
                self.noMoreTrees +=1
                self.treeDetected = False
            #check if new tree dected
            elif 0 < rangeCount < 20 and not self.treeDetected:
                self.state = self.PickerState.PICKING
                self.treeDetected = True
                self.noMoreTrees=0
                print("Found Tree")
                self.addKiwi(time.clock())

    def wait(self):
        #until unloaded
        #while(self.current_load >= self.max_load):
            #do nothing
            time.sleep(1)
