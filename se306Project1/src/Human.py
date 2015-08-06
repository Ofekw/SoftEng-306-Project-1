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

The Human class used to represent a human in the world stage.
It inherits from the Entity class.

"""
class Human(Entity):

    def __init__(self,r_id,x_off,y_off, theta_offset):
        #Insert human specific attirbutes here
        Entity.__init__(self,r_id,x_off,y_off, theta_offset)

    def human_specific_function(self):
        pass
