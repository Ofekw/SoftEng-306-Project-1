#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import random
import numpy.testing
from Human import Human
import ActionInterruptException
import os

"""
@class

Worker class. Inherits from the abstract Human class. The behaviour of this entity will consist
of random movement around the orchard.
"""

class Worker(Human):

    def enum(**enums):
        return type('Enum', (), enums)

    random_location = {}

    random_nav = {}

    WorkerState = enum(PATROLLING_ORCHARD="Moving up and down row",
                        GOING_TO_EMPTY_ORCHARD="Navigating to empty orchard",
                        AVOIDING_ROBOT="Detected robot, leaivng row")


    def __init__(self, r_id, x_off, y_off, theta_offset):
        Human.__init__(self, r_id, x_off, y_off, theta_offset)

        self.worker_state = None

        self.robot_locations = {}

        self.orchard_row_gaps = {}

        self.config = {}

        self.sub_to_picker_positions = rospy.Subscriber("pickerPosition", String, self.Robot_Locations_Callback)
        self.sub_to_carrier_positions = rospy.Subscriber("carrierPosition", String, self.Robot_Locations_Callback)


    def Robot_Locations_Callback(self, message):
        msg = message.data
        msg_values = msg.split(",")

        r_id = msg_values[0]
        r_px = msg_values[1]
        r_py = msg_values[2]

        self.robot_locations[r_id] = [r_px, r_py]

    def go_to_empty_orchard_row(self):
        empty_orchard_row_x = None

        found_row = False

        for g in self.orchard_row_gaps:
            unpopulated = True
            for r_coord in self.robot_locations:
                if (g[0] - 6) <= r_coord[0] <= (g[1] + 6):
                    unpopulated = False

            if (unpopulated):
                empty_orchard_row_x = g[0]
                found_row = True
                break

        if found_row == False:
            self.go_to_empty_orchard_row()

        goto_x_action = self.goto_xy(empty_orchard_row_x, self.py)
        goto_y_action = self.goto_yx(empty_orchard_row_x, -10)
        patrol_action = self.patrol_orchard()

        self._actionsStack_.append(patrol_action)
        self._actionsStack_.append(goto_y_action)
        self._actionsStack_.append(goto_x_action)

    def patrol_orchard(self):
        while self._stopCurrentAction_ == False or self.worker_state != Worker.WorkerState.AVOIDING_ROBOT:
            self.goto_yx(self.px, 40)
            self.goto_yx(self.px, -10)

    def define_orchard_row_gaps(self):
        path_to_config = os.path.abspath(os.path.abspath(os.pardir)) + "/config.properties"

        with open(path_to_config, "r") as f:
            for line in f:
                property = line.split('=')
                self.config[property[0]] = property[1]

        #get number of orchard rows
        rows = int(self.config.get('orchard.number'))

        WORLD_WIDTH = 80
        #max number of orchards is 10
        if rows > 10:
            rows = 10
        elif rows < 1:
            rows = 1
        width_between_rows = WORLD_WIDTH/(rows)

        for x in range(-WORLD_WIDTH/2 + width_between_rows/2,WORLD_WIDTH/2 - width_between_rows/2 + 1, width_between_rows):
            x_left = x + 6
            x_right = x + width_between_rows - 6

            self.orchard_row_gaps.append({x_left, x_right})

    def avoid_robot(self, robot_py):
        if robot_py > self.py:
            self.goto_yx(self.px, -15)
        else:
            self.goto_yx(self.px, 52)

    def check_robot_locations(self):
        for r_coord in self.robot_locations:
            r_px = r_coord[0]
            r_py = r_coord[1]

            if self.px - 3 <= r_px <= self.px + 5:
                if -10 <= r_py <= 48:
                    self.worker_state = Worker.WorkerState.AVOIDING_ROBOT

                    avoid_robot_action = self.avoid_robot(r_py)

                    self._actionsStack_.append(avoid_robot_action)


