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

    def __init__(self,r_id,x_off,y_off):
        self.max_load = 20;
        self.current_load = 0;
        Entity.__init__(self,r_id,x_off,y_off)

    def robot_specific_function(self):
        pass

    def start_picking(self):
       #incomplete
        pass

    def actions(self, rospy):
        #You can use RobotNode_cmdvel to simulate movements, place them in the while loop to try it out
        #RobotNode_cmdvel = geometry_msgs.msg.Twist()

        #moveAction = robot0._actions_[1], [40, 40]
        goToAction = self._actions_[1],[10,20]
        goToAction1 = self._actions_[1],[0,0]
        goToAction2 = self._actions_[1],[10,5]

        self._actionsStack_.append(goToAction)
        self._actionsStack_.append(goToAction1)
        self._actionsStack_.append(goToAction2)

        #check if there is an action on the stack or an action already running
        if(self._actionsStack_.__len__() > 0 and not self._actionRunning_):
            #get top action on stack
            action = self._actionsStack_[-1]
            #run action with parameter
            self._actionRunning_ = True
            result = action[0](*action[1])
            self._actionRunning_=False
            #if action completes succesfully pop it
            if result == 0 or result == 1:
                self._actionsStack_.pop()
