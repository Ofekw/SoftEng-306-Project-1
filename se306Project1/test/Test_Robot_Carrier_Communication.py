#!/usr/bin/env python
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))

import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
from src.RobotCarrier import RobotCarrier

import math
from TestModule import TestModule
import inspect
import subprocess

"""
@class

This is a test for the correct communication of robots and carriers

"""
class Test_Robot_Carrier_Communication(unittest.TestCase,TestModule):

    robot1 = RobotCarrier("CarrierNode",1,0,-28, math.pi/2,50)
    process = (subprocess.Popen(["rosrun", "se306Project1", "Run_RobotPicker.py"], shell=False))

    def test_robot_carrier_communication(self):
        rospy.sleep(5)  # sleep for a while to let the robot picker start moving
        self.assertNotEqual(self.robot1.picker_robots[0], "0,0,0")

    def tearDown(self):
        self.process.kill()

if __name__ == '__main__':
   # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', Test_Robot_Carrier_Communication)