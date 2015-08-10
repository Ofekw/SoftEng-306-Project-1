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

"""
@class

Visitor class. Inherits from the abstract Human class. The behaviour of this entity will consist
of random movement around the orchard.
"""

class Visitor(Human):

    def __init__(self, r_id, x_off, y_off, theta_offset):
        Human.__init__(self, r_id, x_off, y_off, theta_offset)

        self._actions_ = {
            0: self.move_forward,
            1: self.goto,
            2: self.turn,
            3: self.stop,
            4: self.random_nav,
            5: self.go_to_rand_location
        }

        self.linearX = 4

    def StageLaser_callback(self, msg):
        for i in range(80, 120):
            if msg.ranges[i] < 2 and self.disableLaser == False:
                self._stopCurrentAction_ = True
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

        #Randomly select a velocity
        rand_velocity = random.randint(5, 8)

        #Perform movement functions
        self.change_linear_x_to(rand_velocity)
        self.face_direction(rand_direction)
        self.move_forward(rand_dist)

    """
    @function
    This function involves random selecting a coordinate between -40 to 40 x, and -40 to 40 y, then having the entity
    attempt to navigate towards the coordinate. The navigation works by using a goto function to move vertically towards
    an area with no trees or robots (y = -15), then another goto function to move horizontally so the px value lines up to the random
    x value, then finally moving vertically towards the y coordinate.
    """
    def go_to_rand_location(self):

        #Generate random coordinates
        random_x = random.randint(-40, 40)
        random_y = random.randint(-40, 40)

        print("Attempting to go to " + str(random_x) + ", " + str(random_y))

        #Create action that will move vertically to empty area
        move_to_empty_area = self._actions_[1], [self.px, -15]

        #Create action that will move horizontally to line up to x coordinate
        move_to_x = self._actions_[1], [random_x, -15]

        #Create action that will move to random coordinate
        move_to_y = self._actions_[1], [random_x, random_y]

        #Append actions to stack
        self._actionsStack_.append(move_to_y)
        self._actionsStack_.append(move_to_x)
        self._actionsStack_.append(move_to_empty_area)

        #While there are actions on the stack and no action is currently running
        while (len(self._actionsStack_) > 0 and not self._actionRunning_):

            #get top action on stack
            action = self._actionsStack_[-1]

            #set to true
            self._actionRunning_ = True

            #run aciton with paremeter
            result = action[0](*action[1])
            self._stopCurrentAction_ = False

            #if action completes succesfully pop it
            if (result == 0):
                self._actionsStack_.pop()

            self._actionRunning_ = False

    def visitor_specific_function(self):
        if (len(self._actionsStack_) == 0):
            self.go_to_rand_location()

