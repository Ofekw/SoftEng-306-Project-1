#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Animal import Animal
import ActionInterruptException

"""
@MAIN

Main function that creates robot and sets a path

"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running

    robot = Animal(6, -20, -28, math.pi/2)

    while not rospy.is_shutdown():
        robot.animal_specific_function()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
__author__ = 'carvrooom'