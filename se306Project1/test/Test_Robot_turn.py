#!/usr/bin/env python
PKG = 'se306Project1'
NAME = "test_entity_laser_listener"
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
import time
from se306Project1.src.RobotPicker import RobotPicker
import math
import logging
from TestModule import TestModule
import inspect

# A sample python unit test
class Test_Robot_turn(unittest.TestCase,TestModule):

    #Not in 'setUp' because it will be called every time, and that will mean the node will restart its in instantiation
    #since we can't reset the stage, we have to work with the same robot.
    robot0 = RobotPicker(0,-20,-28, math.pi/2)

    def test_turn_right(self):

        self.print_function_name(inspect.stack()[0][3])

        moveAction = self.robot0.turn,["right"]

        self.run_robot(self.robot0,moveAction,5)

        self.assertEqual(self.robot0.get_current_direction(), "east","North Turning Right = EAST")


if __name__ == '__main__':
    unittest.main()