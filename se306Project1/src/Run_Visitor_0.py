#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import numpy.testing
from Visitor import Visitor

"""
@MAIN

Main function that creates robot and sets a path

"""

def main():
    #Construction of Robot objects take 3 params... Robot ID, Start X, Start Y. Start X and Start Y correlates to the myworld.world file
    #Can't create more than one robot per main() .... ie can't run more than one robot per terminal running

    visitor_0 = Visitor(2, 0, -28, 0)

    rospy.Rate(10)
   # rospy.sleep(0.1)

    #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
    #RobotNode_cmdvel = geometry_msgs.msg.Twist()

    random_nav = visitor_0.random_nav, []
    visitor_0._actionsStack_.append(random_nav)

    while not rospy.is_shutdown():

    #check if there is an action on the stack or an action already running
        if(visitor_0._actionsStack_.__len__() > 0 and not visitor_0._actionRunning_):
            #get top action on stack
            action = visitor_0._actionsStack_[-1]
            #run action with parameter
            visitor_0._actionRunning_ = True
            try:
                result = action[0](*action[1])

                visitor_0._actionRunning_=False
                #if action completes succesfully pop it
                if result == 0 or result == 1:
                    visitor_0._actionsStack_.pop()


            except:
                ()
                #turn_left = visitor_0.turn, ["left"]
                #visitor_0._actionsStack_.append(turn_left)

        visitor_0._actionsStack_.append(random_nav)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
