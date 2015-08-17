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

Worker class. Inherits from the abstract Human class. The behaviour of this entity will consist
of random movement around the orchard.
"""

class Worker(Human):

    def enum(**enums):
        return type('Enum', (), enums)

    random_location = {}

    random_nav = {}

    VisitorState = enum(NAVIGATING_RANDOM="Nav to rand location",
                        MOVING_RANDOM = "Move to rand direction")


    def __init__(self, r_id, x_off, y_off, theta_offset):
        Human.__init__(self, r_id, x_off, y_off, theta_offset)

        self.human_state = None

        self.robot_locations = {}

        self.orchard_rows = {}

    def Robot_Location_Callback(self):
        ()

    def go_to_empty_orchard(self):
        ()

    def locate_empty_orchard(self):
        ()

    def patrol_orchard(self):
        ()