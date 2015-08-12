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
import os
import ActionInterruptException


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
        self.kiwi_sub = rospy.Subscriber("picker_kiwiTransfer", String, self.kiwi_callback)
        self.kiwi_pub = rospy.Publisher("carrier_kiwiTransfer",String, queue_size=10)


        self.closestRobotID = 0
        self.carrier_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]
        self.picker_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]

        self.max_load = 100
        self.current_load = 0

        self.is_going_home = False

        Robot.__init__(self,r_id,x_off,y_off,theta_off)


        #these variables are used to help the laser callback, it will help in dealing with entities/debris on
        # it's path to the picker robot
        #self.StageLaser_sub = rospy.Subscriber(self.robot_node_identifier+"/base_scan",sensor_msgs.msg.LaserScan,self.StageLaser_callback)
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

        fn = os.path.join(os.path.dirname(__file__), str(self.robot_id)+"car.sta")
        output_file = open(fn, "w")
        output_file.write(str(self.robot_node_identifier)+ "\n")
        output_file.write("Carrier\n")
        output_file.write(self.state+"\n")
        output_file.write(str(round(self.px,2)) + "\n")
        output_file.write(str(round(self.py,2)) + "\n")
        output_file.write(str(round(self.theta,2)) + "\n")
        output_file.write(str(self.current_load)+ "/" + str(self.max_load))


        #rospy.loginfo("Current x position: %f"picker , self.px)
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
        # print("Picker array")
        # print(', '.join(self.picker_robots))

    """
    @function
    @parameter: message

    Displays info sent from a picker robot when trnsferring kiwis
    """
    def kiwi_callback(self, message):
        if (message.data != str(self.robot_id)):
            self.current_load = 20
            print("going to dropoff zone")
            self.returnToOrigin()

    """
    @function
    @parameter: message

    Tells the picker robot to transfer kiwifruit
    """
    def intiate_transfer(self):
        self.kiwi_pub.publish(str(self.robot_id))
        print("intitate transfer")

    """
    @function
    @parameter: message

    Determines if a change in path needs to take place when moving from a to b.
    Determines what to do when encountering static and dynamic elements.
    """
    def StageLaser_callback(self, msg):
        pass

        # if msg.ranges[90] < 4.0:
        #     self.halt_counter += 1
        #     self._stopCurrentAction_ = True
        #     self.FiveCounter = 0
        # else:
        #     #waits for 5 consecutive not found values, this is to tackle the weird laser scan issue
        #     #that returns alternating incorrect values.
        #     self.FiveCounter += 1
        #     if self.FiveCounter == 5:
        #         self.halt_counter = 0
        #         self._stopCurrentAction_ = False
        #         self.FiveCounter = 0

        #Code for diverting path, which I don't think is needed atm, we can add it later if needed
        # if self.halt_counter == 50:
        #     if not self._divertingPath_:
        #         print "ENCOUNTERED STATIC ELEMENT!!!!"
        #         print "Diverting Path Now..."
        #
        #         move_action = self.move_forward, [10]
        #         turn_action = self.turn, ["left"]
        #         move_forward2 = self.move_forward, [5]
        #         turn_action2 = self.turn, ["right"]
        #
        #         self._actionsStack_.append(move_action)
        #         self._actionsStack_.append(turn_action)
        #         self._actionsStack_.append(move_forward2)
        #         self._actionsStack_.append(turn_action2)
        #         self._stopCurrentAction_ = False
        #     self._divertingPath_ = True
        # elif self.halt_counter == 30:
        #     print "Checking if Entity in front is a static element..."

    """
    @function

    Gets the next robot to collect from
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

    """
    @function
    @parameter: message

    Default action for picker
    Used to wait until a picker is ready for collection
    """
    def waitForPicker(self):
        if self._stopCurrentAction_ == True:
            self._stopCurrentAction_ = False
            raise ActionInterruptException.ActionInterruptException("waitFor Stopped")
        else:
            if not(self.is_going_home):
                self.getClosest()
                if(int(self.picker_robots[self.closestRobotID].split(',')[2]) >= 20):
                    self.goToClosest()

    """
    @function
    @parameter: message

    Go to the full picker
    """
    def goToClosest(self):
        #self._stopCurrentAction_ = True
        #Move robot along x, and then up y
        action = self._actions_[5], [float(self.picker_robots[self.closestRobotID].split(',')[0]), float(self.picker_robots[self.closestRobotID].split(',')[1])-5.0]
            #goto(float(self.closestRobot.split(',')[0]), float(self.closestRobot.split(',')[1]))
        if action != self._actionsStack_[-1]:
            #stop moving foward and add turn action

            # if len(self._actionsStack_) > 0:
            # self._stopCurrentAction_ = True
            self._actionsStack_.append(action)

    """
    @function
    @parameter: message

    Robot has arrived at point, then decides whether to wait or go to drop off
    """
    def arrivedAtPoint(self):
        xabsolute = abs(self.goalx - self.px)
        yabsolute = abs(self.goaly - self.py)
        if (xabsolute < 0.5 and yabsolute < 0.5):
            self.is_going_home = False
            self.current_load = 0
        else:
            self.is_going_home = True

        if (self.is_going_home):
            xgoal = float(self.picker_robots[self.closestRobotID].split(',')[0])
            ygoal = float(self.picker_robots[self.closestRobotID].split(',')[1])
            xabsolute = abs(xgoal - self.px)
            yabsolute = abs(ygoal - self.py)
            if (xabsolute < 0.5 and yabsolute < 5):
                if (int(self.picker_robots[self.closestRobotID].split(',')[2]) == 20):
                    self.intiate_transfer()

    """
    @function
    @parameter: message

    Return to drop off point
    """
    def returnToOrigin(self):
        action = self._actions_[1], [self.init_x, self.init_y]
        self.is_going_home = True;
        self._stopCurrentAction_ = False
        self._actionsStack_.append(action)