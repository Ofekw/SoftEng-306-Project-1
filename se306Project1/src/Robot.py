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
Can go forward and turn left or right or by a certain angle.
"""
class Robot(Entity):

    def __init__(self,r_id,x_off,y_off):

        self.max_load = 20;
        self.current_load = 0;
        Entity.__init__(self,r_id,x_off,y_off)

    def robot_specific_function(self):
        pass

    def start_picking(self):
        while self.current_load < self.max_load:
            #do something
            self.current_load = self.current_load + 1
        pass

"""
@MAIN

Main function that creates robot and sets path
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