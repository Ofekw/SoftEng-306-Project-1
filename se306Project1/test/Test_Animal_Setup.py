#!/usr/bin/env python
import sys
import unittest
import rospy
from std_msgs.msg import *
import sensor_msgs.msg
import time
from se306Project1.src.Animal import Animal
import math
import logging
from TestModule import TestModule
import inspect

class Test_Robot_Setup(unittest.TestCase,TestModule):

    animal = Animal(2,-10,-28, math.pi/2)

    def test_robot_carrier_setup(self):

        self.print_function_name(inspect.stack()[0][3])

        self.assertEqual(self.animal.linearX, 3, "Setting linearX")
        self.assertEqual(self.animal.dict_of_visitors, {}, "Setting dict_of_visitors")
        self.assertEqual(self.animal.animal_state, "", "Setting animal_state")


if __name__ == '__main__':
    unittest.main()