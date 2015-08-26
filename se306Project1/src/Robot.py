#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Entity import Entity

"""
@class

The Robot class used to represent a robot in the world stage.
It inherits from the Entity class.

"""
class Robot(Entity):

    def __init__(self, r_name, r_id, x_off, y_off, theta_off):

        self.current_load = 0
        self.firstLaserReading = []
        self.treesLeft = True
        self.disableSideLaser = False



        Entity.__init__(self, r_name, r_id,x_off,y_off, theta_off)

    """
    @function
    @parameter: Msg msg, Container container

    Method to read laser data into local array
    """
    def read(self, msg, container):
        for i in range(70, 110):
            container.append(msg[i])

    def enum(**enums):
        return type('Enum', (), enums)

    RobotState = enum(PATH="Path finding")