#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from RobotCarrier import RobotCarrier
from Debugger import Debugger
from Robot import Robot
import ActionInterruptException

"""
@MAIN

Main function that creates robot and sets a path

"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running

    robot = RobotCarrier("carrier0", 3, -45, -35, math.pi/2, 30)

    rospy.Rate(100)
    rospy.sleep(0.1)

    #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
    #RobotNode_cmdvel = geometry_msgs.msg.Twist()

    # Add action here to go to position 1
    # moveAction = robot._actions_[1], [robot.px + 20, robot.py]
    moveAction = robot._actions_[6], []
    robot._actionsStack_.append(moveAction)

    while not rospy.is_shutdown():

    #check if there is an action on the stack or an action already running
        if(robot._actionsStack_.__len__() > 0 and not robot._actionRunning_):
            #get top action on stack
            action = robot._actionsStack_[-1]
            #run action with parameter
            robot._actionRunning_ = True
            result = action[0](*action[1])
            robot._actionRunning_=False
            #if action completes succesfully pop it
            robot._stopCurrentAction_ = False
            if result == 0:
                robot._actionsStack_.pop()
                robot.disableSideLaser = False
                if (action[0] == robot._actions_[1] or action[0] == robot._actions_[5] and robot.state != Robot.RobotState.PATH):
                    robot.arrivedAtPoint()
            if result == 2:
                pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
