#!/usr/bin/env python
import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
import time
from se306Project1.src.RobotPicker import RobotPicker
from se306Project1.src.RobotCarrier import RobotCarrier
import math
import logging
from TestModule import TestModule
import inspect

class Test_Robot_Setup(unittest.TestCase,TestModule):

    robot0 = RobotPicker(0,-20,-28, math.pi/2)

    def test_robot_picker_setup(self):

        self.print_function_name(inspect.stack()[0][3])

        #Entity Variables
        self.assertEqual(self.robot0.linearX, 2, "Setting linearX")
        self.assertEqual(self.robot0.angularZ, 0, "Setting angularZ")
        self.assertEqual(self.robot0.init_theta, math.pi/2, "Setting init_theta")
        self.assertEqual(self.robot0.init_x, -20, "Setting init_x")
        self.assertEqual(self.robot0.init_y, -28, "Setting init_y")
        self.assertEqual(self.robot0.px, -20, "Setting px")
        self.assertEqual(self.robot0.py, -28, "Setting py")
        self.assertEqual(self.robot0.robot_id, 0, "Setting robot_id")
        self.assertEqual(self.robot0.robot_node_name, "RobotNode0", "Setting robot_node_name")
        self.assertEqual(self.robot0.robot_node_identifier, "robot_0", "Setting robot_node_identifier")
        self.assertEqual(self.robot0.goalx, -20, "Setting goalx")
        self.assertEqual(self.robot0.goaly, -28, "Setting goaly")
        self.assertEqual(self.robot0.state, self.robot0.State.STOPPED, "Setting state")
        self.assertEqual(self.robot0._actionsStack_, [], "Setting _actionsStack_")
        self.assertEqual(self.robot0._stopCurrentAction_, False, "Setting _stopCurrentAction_")
        self.assertEqual(self.robot0._actionRunning_, False, "Setting _actionRunning_")
        self.assertEqual(self.robot0.disableLaser, False, "Setting disableLaser")
        self.assertEqual(self.robot0.noMoreTrees, 0, "Setting noMoreTrees")
        self.assertEqual(self.robot0.treeDetected, False, "Setting treeDetected")
        self.assertEqual(self.robot0.atOrchard, False, "Setting atOrchard")
        #Robot Picker Variables
        self.assertEqual(self.robot0.max_load, 20, "Setting max_load")
        self.assertEqual(self.robot0.current_load, 0, "Setting current_load")
        self.assertEqual(self.robot0.firstLaserReading, [], "Setting firstLaserReading")

if __name__ == '__main__':
    unittest.main()