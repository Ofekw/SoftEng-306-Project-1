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

Main function that creates robot and sets a path

"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running

    robot0 = Robot(0,0,0)

    rospy.Rate(100)
    rospy.sleep(0.1)

    print("Current x pos = " + str(robot0.px))
    print("Current y pos = " + str(robot0.py))


    #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
    #RobotNode_cmdvel = geometry_msgs.msg.Twist()

    # while not rospy.is_shutdown():

    robot0.get_distance(5,20)
    robot0.goto(5,20)
    print("Arrived at destination:", robot0.px, robot0.py)
    robot0.get_distance(0,5)
    robot0.goto(0,5)
    print("Arrived at destination:", robot0.px, robot0.py)
    robot0.get_distance(0,0)
    robot0.goto(0,0)
    print("Arrived at destination:", robot0.px, robot0.py)

    # robot0.rotate_relative(80,"degrees")
    # print("current theta:" + str(robot0.theta))
    # robot0.correct_theta()
    # print("current theta:" + str(robot0.theta))



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass