#!/usr/bin/env python
# PKG = 'se306Project1'
# NAME = "test_robot_go_to"
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))
import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
import time
from src.RobotPicker import RobotPicker
import math
import logging
from TestModule import TestModule
import inspect


class Test_Robot_goto_xy(unittest.TestCase,TestModule):

    #Not in 'setUp' because it will be called every time, and that will mean the node will restart its in instantiation
    #since we can't reset the stage, we have to work with the same robot.
    robot0 = RobotPicker(0,-20,-28, math.pi/2)

    def test_goto_xy_1move_right(self):

        self.print_function_name(inspect.stack()[0][3])

        end_x = -15
        end_y = -28

        moveAction = self.robot0.goto_xy, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_xy_2move_left(self):

        self.print_function_name(inspect.stack()[0][3])

        end_x = -30
        end_y = -28

        moveAction = self.robot0.goto_xy, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_xy_3move_up(self):

        self.print_function_name(inspect.stack()[0][3])

        end_x = -30
        end_y = -20

        moveAction = self.robot0.goto_xy, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_xy_4move_down(self):

        self.print_function_name(inspect.stack()[0][3])

        end_x = -30
        end_y = -30

        moveAction = self.robot0.goto_xy, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

if __name__ == '__main__':
    unittest.main()
    # import rostest
    # rostest.rosrun("se306Project1", "test_robot_goto", TestRobotGoTo)