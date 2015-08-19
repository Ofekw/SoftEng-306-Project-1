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

class Test_Robot_Setup(unittest.TestCase,TestModule):

    robot1 = RobotCarrier(1,-10,-28, math.pi/2)

    def test_robot_carrier_setup(self):

        self.print_function_name(inspect.stack()[0][3])

        #Entity Variables
        self.assertEqual(self.robot1.linearX, 2, "Setting linearX")
        self.assertEqual(self.robot1.angularZ, 0, "Setting angularZ")
        self.assertEqual(self.robot1.init_theta, math.pi/2, "Setting init_theta")
        self.assertEqual(self.robot1.init_x, -10, "Setting init_x")
        self.assertEqual(self.robot1.init_y, -28, "Setting init_y")
        self.assertEqual(self.robot1.px, -10, "Setting px")
        self.assertEqual(self.robot1.py, -28, "Setting py")
        self.assertEqual(self.robot1.robot_id, 1, "Setting robot_id")
        self.assertEqual(self.robot1.robot_node_name, "RobotNode1", "Setting robot_node_name")
        self.assertEqual(self.robot1.robot_node_identifier, "robot_1", "Setting robot_node_identifier")
        self.assertEqual(self.robot1.goalx, -10, "Setting goalx")
        self.assertEqual(self.robot1.goaly, -28, "Setting goaly")
        self.assertEqual(self.robot1.state, self.robot1.State.STOPPED, "Setting state")
        self.assertEqual(self.robot1._actionsStack_, [], "Setting _actionsStack_")
        self.assertEqual(self.robot1._stopCurrentAction_, False, "Setting _stopCurrentAction_")
        self.assertEqual(self.robot1._actionRunning_, False, "Setting _actionRunning_")
        self.assertEqual(self.robot1.disableLaser, False, "Setting disableLaser")
        self.assertEqual(self.robot1.noMoreTrees, 0, "Setting noMoreTrees")
        self.assertEqual(self.robot1.treeDetected, False, "Setting treeDetected")
        self.assertEqual(self.robot1.atOrchard, False, "Setting atOrchard")
        #Robot Carrier Variables
        self.assertEqual(self.robot1.closestRobotID, 0, "Setting closestRobotID")
        self.assertEqual(self.robot1.nextRobotID, 0, "Setting nextRobotID")
        self.assertEqual(self.robot1.carrier_robots, ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"], "Setting carrier_robots")
        self.assertEqual(self.robot1.picker_robots, ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"], "Setting picker_robots")
        self.assertEqual(self.robot1.max_load, 100, "Setting max_load")
        self.assertEqual(self.robot1.current_load, 0, "Setting current_load")
        self.assertEqual(self.robot1.is_going_home, False, "Setting is_going_home")

        pass


if __name__ == '__main__':
    unittest.main()