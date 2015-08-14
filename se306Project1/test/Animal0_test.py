#!/usr/bin/env python

import rospy
import math
from se306Project1.src.Animal import Animal



"""
@MAIN

Main function that creates robot and sets a path

"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running

    robot = Animal(5, 20, -28, math.pi/2)

    while not rospy.is_shutdown():
        robot.animal_specific_function()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
__author__ = 'carvrooom'
