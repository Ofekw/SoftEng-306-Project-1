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

    def __init__(self, r_id, x_off, y_off, theta_off):

        self.max_load = 20
        self.current_load = 0
        Entity.__init__(self,r_id,x_off,y_off, theta_off)

    def robot_specific_function(self):
        pass

    def start_picking(self):
       #incomplete
        pass