#!/usr/bin/env python
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))
import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
from src.RobotPicker import RobotPicker
import math
from TestModule import TestModule
import inspect

class Test_Robot_Setup(unittest.TestCase,TestModule):

    robot0 = RobotPicker(0,-20,-28, math.pi/2)

    def test_robot_picker_setup(self):

        self.print_function_name(inspect.stack()[0][3])

        #Entity Variables
        self.assertEqual(self.robot0.linearX, 2)
        self.assertEqual(self.robot0.angularZ, 0)
        self.assertEqual(self.robot0.init_theta, math.pi/2)
        self.assertEqual(self.robot0.init_x, -20)
        self.assertEqual(self.robot0.init_y, -28)
        self.assertEqual(self.robot0.px, -20)
        self.assertEqual(self.robot0.py, -28)
        self.assertEqual(self.robot0.robot_id, 0)
        self.assertEqual(self.robot0.robot_node_name,"RobotNode0")
        self.assertEqual(self.robot0.robot_node_identifier,"robot_0")
        self.assertEqual(self.robot0.goalx, -20)
        self.assertEqual(self.robot0.goaly, -28)
        self.assertEqual(self.robot0.state, "Stopped")
        self.assertEqual(self.robot0._actionsStack_, [])
        self.assertEqual(self.robot0._stopCurrentAction_, False)
        self.assertEqual(self.robot0._actionRunning_, False)
        self.assertEqual(self.robot0.disableLaser, False)
        self.assertEqual(self.robot0.noMoreTrees, 0)
        self.assertEqual(self.robot0.treeDetected, False)
        self.assertEqual(self.robot0.atOrchard, False)
        #Robot Picker Variables
        self.assertEqual(self.robot0.max_load, 50)
        self.assertEqual(self.robot0.current_load, 0)
        self.assertEqual(self.robot0.firstLaserReading, [])

if __name__ == '__main__':
    unittest.main()