#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Worker import Worker
import ActionInterruptException
import time

"""
@MAIN

Main function that creates robot and sets a path

"""

def main():
    robot = Worker("Worker0",2, 10, -20, 0)
    rospy.sleep(5)

    while not rospy.is_shutdown():
        robot.worker_specific_function()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
