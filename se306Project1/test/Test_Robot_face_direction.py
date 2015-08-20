#!/usr/bin/env python
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

class Test_Robot_face_direction(unittest.TestCase,TestModule):

    robot0 = RobotPicker(0,-20,-28, math.pi/2)

    def test_face_south(self):

        self.print_function_name(inspect.stack()[0][3])

        moveAction = self.robot0.face_direction,["south"]

        self.assertTrue(self.run_robot(self.robot0,moveAction,10), "Time Limit Exceeded")

        self.assertEqual(self.robot0.get_current_direction(), "south","Turning to face south")

    def test_face_east(self):

        self.print_function_name(inspect.stack()[0][3])

        moveAction = self.robot0.face_direction,["east"]

        self.assertTrue(self.run_robot(self.robot0,moveAction,10), "Time Limit Exceeded")

        self.assertEqual(self.robot0.get_current_direction(), "east","Turning to face south")

    def test_face_north(self):

        self.print_function_name(inspect.stack()[0][3])

        moveAction = self.robot0.face_direction,["north"]

        self.assertTrue(self.run_robot(self.robot0,moveAction,10), "Time Limit Exceeded")

        self.assertEqual(self.robot0.get_current_direction(), "north","Turning to face south")

    def test_face_west(self):

        self.print_function_name(inspect.stack()[0][3])

        moveAction = self.robot0.face_direction,["west"]

        self.assertTrue(self.run_robot(self.robot0,moveAction,10), "Time Limit Exceeded")

        self.assertEqual(self.robot0.get_current_direction(), "west","Turning to face south")


if __name__ == '__main__':
    unittest.main()