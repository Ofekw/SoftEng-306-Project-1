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
from src.RobotCarrier import RobotCarrier
import math
import logging
from TestModule import TestModule
import inspect

class Test_Robot_Carrier_Setup(unittest.TestCase,TestModule):

    robot1 = RobotCarrier(1,-10,-28, math.pi/2)

    def test_robot_carrier_setup(self):

        #Entity Variables
        self.assertEqual(self.robot1.linearX, 2)
        self.assertEqual(self.robot1.angularZ, 0)
        self.assertEqual(self.robot1.init_theta, math.pi/2)
        self.assertEqual(self.robot1.init_x, -10)
        self.assertEqual(self.robot1.init_y, -28)
        self.assertEqual(self.robot1.px, -10)
        self.assertEqual(self.robot1.py, -28)
        self.assertEqual(self.robot1.robot_id, 1)
        self.assertEqual(self.robot1.robot_node_name, "RobotNode1")
        self.assertEqual(self.robot1.robot_node_identifier, "robot_1")
        self.assertEqual(self.robot1.goalx, -10)
        self.assertEqual(self.robot1.goaly, -28)
        self.assertEqual(self.robot1.state, "Stopped")
        self.assertEqual(self.robot1._actionsStack_, [])
        self.assertEqual(self.robot1._stopCurrentAction_, False)
        self.assertEqual(self.robot1._actionRunning_, False)
        self.assertEqual(self.robot1.disableLaser, False)
        self.assertEqual(self.robot1.noMoreTrees, 0)
        self.assertEqual(self.robot1.treeDetected, False)
        self.assertEqual(self.robot1.atOrchard, False)
        #Robot Carrier Variables
        self.assertEqual(self.robot1.next_robot_id, None)
        self.assertEqual(self.robot1.carrier_robots, ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"])
        self.assertEqual(self.robot1.picker_robots, ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"])
        self.assertEqual(self.robot1.max_load, 50)
        self.assertEqual(self.robot1.current_load, 0)
        self.assertEqual(self.robot1.is_going_home, False)

        pass


if __name__ == '__main__':
    # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', Test_Robot_Carrier_Setup)