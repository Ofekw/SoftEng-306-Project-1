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

    def go_to_rand_location(self):
        random_x = random.randint(-40, 40)
        random_y = random.randint(-40, 40)

        move_to_empty_area = self._actions_.goto, [self.px, -18]

        move_to_x = self._actions_.goton, [random_x, self.py]

        move_to_y = self._actions_.goto, [self.px, random_y]

        self._actionsStack_.append(move_to_y)
        self._actionsStack_.append(move_to_x)
        self._actionsStack_.append(move_to_empty_area)



    def visitor_specific_function(self):
        self.random_nav

