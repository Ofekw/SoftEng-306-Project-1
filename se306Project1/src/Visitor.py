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
"""

class Visitor(Human):

    def enum(**enums):
        return type('Enum', (), enums)

    random_location = {}

    random_nav = {}

    VisitorState = enum(NAVIGATING_RANDOM="Nav to rand location",
                        MOVING_RANDOM = "Move to rand direction")


    def __init__(self, r_name, r_id, x_off, y_off, theta_offset):
        Human.__init__(self, r_name, r_id, x_off, y_off, theta_offset)

        self.pub_to_dog = rospy.Publisher("visitor_dog_topic", String, queue_size=10)

        self.visitor_state = ""


        self._actions_ = {
            0: self.move_forward,
            1: self.goto_yx,
            2: self.turn,
            3: self.stop,
            4: self.random_nav,
            5: self.go_to_rand_location
        }

        self.linearX = 3

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


    def StageLaser_callback(self, msg):
        for i in range(60, 120):
            if (msg.ranges[i] < 4 and self.disableLaser == False):
                self._stopCurrentAction_ = True
                move1 = self._actions_[0], [3]
                turn2 = self._actions_[2], ["right"]
                self._actionsStack_.append(move1)
                self._actionsStack_.append(turn2)

                return


    """
    @function
    This function when called will cause the Visitor entity to turn a random cardinal direction, then move forward
    a random distance between 5 and 10m, at a random velocity between 2 and 4 m/s
    """
    def random_nav(self):
        global random_nav
        #Create an array of the cardinal directions
        cardinal_directions = ["north", "south", "west", "east"]

        #Randomly select a direction
        rand_direction = cardinal_directions[random.randint(0, 3)]

        #Random select a distance to move forward
        rand_dist = random.randint(15, 30)

        #random_nav[0] = rand_direction
        #random_nav[1] = str(rand_dist)
        self.visitor_state = self.VisitorState.MOVING_RANDOM

        self.face_direction(rand_direction)
        self.move_forward(rand_dist)

    """
    @function
    This function involves randomly selecting a coordinate between -40 to 40 x, and -40 to 40 y, then having the entity
    attempt to navigate towards the coordinate. The navigation works by using a goto function to move vertically towards
    an area with no trees or robots (y = -15), then another goto function to move horizontally so the px value lines up to the random
    x value, then finally moving vertically towards the y coordinate.
    """
    def go_to_rand_location(self):
        global random_location

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


    def visitor_specific_function(self):
        random_action_int = random.randint(0, 10)

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
                #run aciton with paremeter
                result = action[0](*action[1])

                del self._actionsStack_[self._actionsStack_.index(action)]

                self._actionRunning_ = False
            except ActionInterruptException.ActionInterruptException as e:
                ()

