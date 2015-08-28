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

"""
@class

This is a test for the ability of the function goto_yx in a more complex sense, using diagonal travels

"""
class Test_Robot_goto_yx_complex(unittest.TestCase,TestModule):

    #Not in 'setUp' because it will be called every time, and that will mean the node will restart its in instantiation
    #since we can't reset the stage, we have to work with the same robot.
    robot0 = RobotPicker("Node",0,-20,-28, math.pi/2,50)

    def test_goto_yx_1move_up_right(self):

        end_x = -15
        end_y = -23

        moveAction = self.robot0.goto_yx, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_yx_2move_down_left(self):

        end_x = -20
        end_y = -28

        moveAction = self.robot0.goto_yx, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_yx_3move_up_left(self):

        end_x = -25
        end_y = -23

        moveAction = self.robot0.goto_yx, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

    def test_goto_yx_4move_down_right(self):

        end_x = -20
        end_y = -28

        moveAction = self.robot0.goto_yx, [end_x,end_y]

        self.run_robot(self.robot0,moveAction,15)

        self.assertTrue(self.compare_values_with_threshold(self.robot0.px,end_x))
        self.assertTrue(self.compare_values_with_threshold(self.robot0.py,end_y))

if __name__ == '__main__':
    # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', Test_Robot_goto_yx_complex)