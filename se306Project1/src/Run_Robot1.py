#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Robot import Robot

"""
@MAIN

Main function that creates a robot and sets a path

"""

def main():
    #insert instantiation and path moving here
    pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass