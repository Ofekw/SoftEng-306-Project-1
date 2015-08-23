#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import random
import numpy.testing
from Entity import Entity
from Human import Human
import ActionInterruptException
import os

"""
@class Animal

Animal class. Inherits from the Entity class. Its behaviour consists of either random movement around the orchard, or
going to the last known location of a visitor. It gets the location of the visitor by subscribing to the visitor_dog_topic.
"""

class Animal(Entity):

    def enum(**enums):
        return type('Enum', (), enums)

    AnimalState = enum(NAVIGATING_RANDOM = "Nav to rand location",
                        GOING_TO_VISITOR = "Going to visitor")


    def __init__(self, r_name, r_id, x_off, y_off, theta_offset):
        Entity.__init__(self, r_name, r_id, x_off, y_off, theta_offset)

        self.dict_of_visitors = {}

        #Initialise subscriber to visitor
        self.sub_to_visitor = rospy.Subscriber("visitor_dog_topic", String, self.Visitor_Subscription)

        self.linearX = 3

        self.animal_state = ""

        self._actions_ = {
            0: self.move_forward,
            1: self.goto_yx,
            2: self.turn,
            3: self.stop,
            4: self.go_to_rand_location,
            5: self.go_to_visitor
        }

    """
    @function

    Call back function to update position values. Also will write current state information to a wor.sta file which
    is to be used by the GUI
    """
    def StageOdom_callback(self, msg):
        #Update the px and py values
        self.update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

        #Find the yaw from the quaternion values
        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        #Update the theta value
        self.update_theta(yaw)

        fn = os.path.join(os.path.dirname(__file__), str(self.robot_id)+"ani.sta")
        output_file = open(fn, "w")
        output_file.write(str(self)+str(self.robot_id)+ "\n")
        output_file.write("Animal\n")
        output_file.write(self.animal_state + "\n")
        output_file.write(str(round(self.px,2)) + "\n")
        output_file.write(str(round(self.py,2)) + "\n")
        output_file.write(str(round(self.theta,2)) + "\n")
        output_file.close()


    """
    @function

    Call back function for the laser messages. When this entity detects a object in front of it,
    it will stop current action, then turn right and move forward.
    """
    def StageLaser_callback(self, msg):
        #Define objects being in front of the Animal as being within a 30 degree radius
        for i in range(60, 120):
            #If object within 3m and the laser is not disabled
            if (msg.ranges[i] < 4 and self.disableLaser == False):
                #Stop current action
                self._stopCurrentAction_ = True

                #Create a move and turn action
                move1 = self._actions_[0], [3]
                turn2 = self._actions_[2], ["right"]

                #Append actions to stack
                self._actionsStack_.append(move1)
                self._actionsStack_.append(turn2)

                return

    """
    @function

    Callback function that is invoked when the Visitor publishes its location. It will store the location
    into a dictionary of visitor locations.
    """
    def Visitor_Subscription(self, visitor_message):

        #Obtain the message and the corresponding message values
        msg = "" + visitor_message.data
        visitor_values = msg.split(":", 3)

        v_id = visitor_values[0]
        v_x = float(visitor_values[1])
        v_y = float(visitor_values[2])

        position = [v_x, v_y]

        #Place the location into the dictionary using the visitor id as the key
        self.dict_of_visitors[v_id] = position


    """
    @function

    Create a set of goto functions that will move to the given coordinate
    """
    def go_to_location(self, x, y):

        print("Attempting to go to " + str(x) + ", " + str(y))

        #Create action that will move vertically to empty area
        move_to_empty_area = self._actions_[1], [self.px, -15]

        #Create action that will move horizontally to line up to x coordinate
        move_to_x = self._actions_[1], [x, -15]

        #Create action that will move to random coordinate
        move_to_y = self._actions_[1], [x, y]

        #Append actions to stack
        #If current y location is greater than -15, then append the move to empty area function
        if (self.py > -15):
            self._actionsStack_.append(move_to_empty_area)

        self._actionsStack_.append(move_to_x)
        self._actionsStack_.append(move_to_y)

    """
    @function

    Go to a randomly generated coordinate
    """
    def go_to_rand_location(self):
        #Generate random x and y coordinate
        random_x = random.randint(-40, 40)
        random_y = random.randint(-40, 40)

        #Set state
        self.animal_state = self.AnimalState.NAVIGATING_RANDOM

        self.go_to_location(random_x, random_y)

    """
    @function

    Obtain a coordinate value from the dictionary of visitor coordinates. Then use the go_to_location to navigate
    to that coordinate
    """
    def go_to_visitor(self):

        if (self.dict_of_visitors.__len__() > 0):
            self.animal_state = self.AnimalState.GOING_TO_VISITOR

            #Obtain any value from the visitor dictionary
            coord = self.dict_of_visitors.values()[0]
            visitor_x = coord[0]
            visitor_y = coord[1]

            self.go_to_location(visitor_x, visitor_y)

    """
    @function

    Function that will be repeatedly called in the main while loop of the Run_Animal.py script.
    
    """
    def animal_specific_function(self):

        random_action_int = random.randint(0, 10)

        if (random_action_int < 5):
            for i in range(0, 10):
                action_init = self.go_to_visitor, []
                self._actionsStack_.append(action_init)
        else:
            action_init = self.go_to_rand_location, []
            self._actionsStack_.append(action_init)

        #While there are actions on the stack and no action is currently running
        while (len(self._actionsStack_) > 0 and not self._actionRunning_):

            #get top action on stack
            action = self._actionsStack_[-1]

            try:
                self._actionRunning_ = True
                #run aciton with paremeter
                result = action[0](*action[1])

                del self._actionsStack_[self._actionsStack_.index(action)]
                self._actionRunning_ = False

            #Catch the exception that will be raised when the stopCurrentAction is set to True, then delete last action
            #from stack
            except ActionInterruptException.ActionInterruptException as e:
                #Remove the last currently ran action from the stack
                del self._actionsStack_[self._actionsStack_.index(action)]
                self._actionRunning_ = False

    def __str__(self):
        return "animal_"