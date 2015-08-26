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
from src.Animal import Animal
import math
import logging
from TestModule import TestModule
import inspect

class Test_Robot_Setup(unittest.TestCase,TestModule):

    animal = Animal("Node",2,-10,-28, math.pi/2)

    def test_robot_carrier_setup(self):

        self.assertEqual(self.animal.linearX, 3, "Setting linearX")
        self.assertEqual(self.animal.dict_of_visitors, {}, "Setting dict_of_visitors")
        self.assertEqual(self.animal.animal_state, "", "Setting animal_state")


if __name__ == '__main__':
    # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', Test_Robot_Setup)