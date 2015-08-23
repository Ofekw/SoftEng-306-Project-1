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
from Human import Human
import ActionInterruptException
import os

"""
@class

Visitor class. Inherits from the abstract Human class. The behaviour of this entity will consist
of random movement around the orchard.

It also publishes to a topic named "visitor_dog_topic". It sends a String that holds coordinate information
about the Visitor, which the Animal entity subscribes to.
"""

class Visitor(Human):

    def enum(**enums):
        return type('Enum', (), enums)

    VisitorState = enum(NAVIGATING_RANDOM="Nav to rand location",
                        MOVING_RANDOM = "Move to rand direction")


    def __init__(self, r_name, r_id, x_off, y_off, theta_offset):
        Human.__init__(self, r_name, r_id, x_off, y_off, theta_offset)

        #Initialise the publisher
        self.pub_to_dog = rospy.Publisher("visitor_dog_topic", String, queue_size=10)

        #Set the state to initially empty
        self.visitor_state = ""

        #Set the actions to be used by the Visitor class
        self._actions_ = {
            0: self.move_forward,
            1: self.goto_yx,
            2: self.turn,
            3: self.stop,
            4: self.random_nav,
            5: self.go_to_rand_location,
            6: self.face_direction
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

        self.pub_to_dog.publish(str(self.robot_id) + ":" + str(self.px) + ":" + str(self.py))

        fn = os.path.join(os.path.dirname(__file__), str(self.robot_id)+"vis.sta")
        output_file = open(fn, "w")
        output_file.write(str(self)+str(self.robot_id)+ "\n")
        output_file.write("Visitor\n")
        output_file.write(self.visitor_state + "\n")
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
        #Check for a degree range between 60 and 120 degrees
        for i in range(60, 120):
            #If object within 4m and not already turning then perform collision avoidance
            if (msg.ranges[i] < 4 and self.disableLaser == False):

                #Stop current action
                self._stopCurrentAction_ = True

                #Create actions to turn right and move forward
                move1 = self._actions_[0], [3]
                turn2 = self._actions_[2], ["right"]

                #Append actions to stack
                self._actionsStack_.append(move1)
                self._actionsStack_.append(turn2)

                return


    """
    @function
    This function when called will cause the Visitor entity to turn a random cardinal direction, then move forward
    a random distance between 5 and 10m, at a random velocity between 2 and 4 m/s
    """
    def random_nav(self):
        #Create an array of the cardinal directions
        cardinal_directions = ["north", "south", "west", "east"]

        #Randomly select a direction
        rand_direction = cardinal_directions[random.randint(0, 3)]

        #Random select a distance to move forward
        rand_dist = random.randint(15, 30)

        #random_nav[0] = rand_direction
        #random_nav[1] = str(rand_dist)
        self.visitor_state = self.VisitorState.MOVING_RANDOM

        # face_rand_direction = self._actions_[6], [rand_direction]
        # move_forward = self._actions_[0], [rand_dist]
        #
        # self._actionsStack_.append(move_forward)
        # self._actionsStack_.append(face_rand_direction)

        self.face_direction(rand_direction)
        self.move_forward(rand_dist)

    """
    @function.
    This function involves randomly selecting a coordinate between -40 to 40 x, and -40 to 40 y, then having the entity
    attempt to navigate towards the coordinate. The navigation works by using a goto function to move vertically towards
    an area with no trees or robots (y = -15), then another goto function to move horizontally so the px value lines up to the random
    x value, then finally moving vertically towards the y coordinate.
    """
    def go_to_rand_location(self):

        #Generate random coordinates
        random_x = random.randint(-40, 40)
        random_y = random.randint(-40, 40)

        random_location = {random_x, random_y}
        self.visitor_state = self.VisitorState.NAVIGATING_RANDOM

        print("Attempting to go to " + str(random_x) + ", " + str(random_y))

        #Create action that will move vertically to empty area
        move_to_empty_area = self._actions_[1], [self.px, -15]

        #Create action that will move horizontally to line up to x coordinate
        move_to_x = self._actions_[1], [random_x, -15]

        #Create action that will move to random coordinate
        move_to_y = self._actions_[1], [random_x, random_y]

        #Append actions to stack
        #If current y location is greater than -15, then append the move to empty area function

        self._actionsStack_.append(move_to_y)
        self._actionsStack_.append(move_to_x)

        if (self.py > -15):
            self._actionsStack_.append(move_to_empty_area)

    """
    @function

    Function to be called by the main while loop in the Run_Visitor scripts. It will set the initial action
    to either random_nav or go_to_rand_location.
    """
    def visitor_specific_function(self):
        #Generate a random integer to be used between 0 and 10
        random_action_int = random.randint(0, 10)

        #50% chance of being either random_nav or go_to_rand_location as first action
        if (random_action_int < 6):
            action_init = self.random_nav, []
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


