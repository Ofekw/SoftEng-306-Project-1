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
from collections import deque
import threading


"""
@class

The Robot class used to represent a robot in the world stage.
It inherits from the Entity class.

"""
class RobotCarrier(Robot):

    def enum(**enums):
        return type('Enum', (), enums)

    CarrierState = enum(WAITINGFORPICKER="Waiting for picker",
                              GOINGTOPICKER="Going to picker", GOTODROPOFF="Going to dropoff")

    def __init__(self,r_id,x_off,y_off,theta_off):

        Robot.__init__(self,r_id,x_off,y_off,theta_off)
        #globals.init()

        self.carrier_pub = rospy.Publisher("carrier_position",String, queue_size=10)
        self.carrier_sub = rospy.Subscriber("carrier_position", String, self.carrier_callback)
        self.picker_sub = rospy.Subscriber("picker_position", String, self.picker_callback)
        self.kiwi_sub = rospy.Subscriber("picker_kiwi_transfer", String, self.kiwi_callback)
        self.kiwi_pub = rospy.Publisher("carrier_kiwi_transfer",String, queue_size=10)
        self.queue_pub = rospy.Publisher("carrier_allocation_request", String, queue_size=10)
        self.queue_sub = rospy.Subscriber("carrier_allocation_response", String, self.queue_callback)

        self.next_robot_id = None
        self.carrier_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]
        self.picker_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]

        # self.max_load = 100
        # self.current_load = 0

        self.is_going_home = False

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

        #Sending it's location to the topic
        self.carrier_pub.publish(str(self.robot_id) + "," + str(self.px) + "," + str(self.py) + "," + str(self.theta))

        fn = os.path.join(os.path.dirname(__file__), str(self.robot_id)+"car.sta")
        output_file = open(fn, "w")
        output_file.write(str(self.robot_node_identifier)+ "\n")
        output_file.write("Carrier\n")
        output_file.write(self.state+"\n")
        output_file.write(str(round(self.px,2)) + "\n")
        output_file.write(str(round(self.py,2)) + "\n")
        output_file.write(str(round(self.theta,2)) + "\n")
        output_file.write(str(self.current_load)+ "/" + str(self.max_load))
        output_file.close()
        

    """
    @function
    @parameter: message

    Sets the position of carrier robots received from messages on the topic
    """
    def carrier_callback(self, message):
        self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2] #+ "," + message.data.split(',')[4]  # Should add element 4 here which is theta
        # print("Carrier array")
        # print ', '.join(self.carrier_robots)

    """
    @function
    @parameter: message

    Sets the position of picker robots received from messages on the topic
    """
    def picker_callback(self, message):
        picker_index = int(message.data.split(',')[0])
        self.picker_robots[picker_index] = message.data.split(',')[1] + "," + message.data.split(',')[2] + "," + message.data.split(',')[4]  # Should add element 3 here which is theta

    """
    @function
    @parameter: message

    Callback from the picker for confirmation of transfer
    Carrier publishes to the carrier_queue that it has completed
    """
    def kiwi_callback(self, message):
        # if the id of the picker robot == the robot id it is supposed to go to
        if (int(message.data) == self.next_robot_id):
            self.current_load = self.max_load # possible add max load here

            # signal queue that transfer has completed
            self.queue_pub.publish(str(self.robot_id) + ",arrived," + str(self.next_robot_id))
            self.next_robot_id = None
            print("going to dropoff zone")
            self.returnToOrigin()

    """
    @function
    @parameter: message

    Callback from carrier_queue to let the carrier know which robot to go to next
    Then goes to that robot if it is not going home
    """
    def queue_callback(self, message):
        #self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2] #+ "," + message.data.split(',')[4]  # Should add element 4 here which is theta
        if(int(message.data.split(',')[0]) == self.robot_id):
            self.next_robot_id = int(message.data.split(',')[1])
            if not(self.is_going_home):
                self.go_to_next_picker()

    """
    @function
    @parameter: message

    Tells the picker robot to transfer kiwifruit
    """
    def initiate_transfer(self):
        self.kiwi_pub.publish(str(self.robot_id) + "," + str(self.next_robot_id))
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
    @parameter: message

    Default action for picker
    Constantly publishes itself to carrier_pub and asks the carrier_queue if there is something to do
    """
    def waitForPicker(self):
        self.state = self.CarrierState.WAITINGFORPICKER
        self.carrier_pub.publish(str(self.robot_id) + "," + str(self.px) + "," + str(self.py) + "," + str(self.theta))
        if self._stopCurrentAction_ == True:
            self._stopCurrentAction_ = False
            raise ActionInterruptException.ActionInterruptException("waitFor Stopped")
        else:
            if not(self.is_going_home):
                if(self.next_robot_id == None):
                    self.queue_pub.publish(str(self.robot_id) + ",waiting,"  + str(self.next_robot_id))
                    rospy.sleep(0.5) # without the sleep it publishes many times making the system slow down


    """
    @function
    @parameter: message

    Go to the next picker as denoted by next_robot_id
    """
    def go_to_next_picker(self):
        self.state = self.CarrierState.GOINGTOPICKER
        action = self._actions_[5], [float(self.picker_robots[self.next_robot_id].split(',')[0]), float(self.picker_robots[self.next_robot_id].split(',')[1])-5.0]
        if action != self._actionsStack_[-1]:
            self._actionsStack_.append(action)

    """
    @function
    @parameter: message

    Robot has arrived at point, then decides if it has arrived at the picker or at home
    If at home, it finishes and waitForPicker is called again
    If at picker, it initiates transfer
    """
    def arrivedAtPoint(self):
        xabsolute = abs(self.goalx - self.px)
        yabsolute = abs(self.goaly - self.py)
        if (xabsolute < 0.5 and yabsolute < 0.5):
            self.is_going_home = False
            self.current_load = 0
        else:
            self.is_going_home = True

        if (self.is_going_home):    # may be redundent check?
            xgoal = float(self.picker_robots[self.next_robot_id].split(',')[0])
            ygoal = float(self.picker_robots[self.next_robot_id].split(',')[1])
            xabsolute = abs(xgoal - self.px)
            yabsolute = abs(ygoal - self.py)
            if (xabsolute < 2 and yabsolute < 6):
                if (int(self.picker_robots[self.next_robot_id].split(',')[2]) == self.max_load):
                    self.initiate_transfer()

    """
    @function
    @parameter: message

    Return to drop off point
    """
    def returnToOrigin(self):
        self.state = self.CarrierState.GOTODROPOFF
        action = self._actions_[1], [self.init_x, self.init_y]
        self.is_going_home = True;
        self._stopCurrentAction_ = False
        self._actionsStack_.append(action)

    def printLists(self):
        #print("picker queue is " + str(picker_queue))
        #print("targeted queue is " + str(targeted_pickers))
        pass
