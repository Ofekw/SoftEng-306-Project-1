#!/usr/bin/env python

import rospy
import time
import inspect

"""
@class

This is a parent class to all our test cases. It provides beneficial functions that will reduce amounts of code in all our tests.

"""
class TestModule():

    def run_robot(self, robot0, action , time_limit):

        max_time = time.time() + time_limit
        robot0._actionsStack_.append(action)

        counter = 0

        while (not rospy.is_shutdown()) and time.time() < max_time:

            #check if there is an action on the stack or an action already running
            if(robot0._actionsStack_.__len__() > 0 and not robot0._actionRunning_):
                #get top action on stack
                action = robot0._actionsStack_[-1]
                #run action with parameter
                robot0._actionRunning_ = True
                result = action[0](*action[1])
                robot0._actionRunning_=False
                #if action completes succesfully pop it
                if result == 0 or result == 1:
                    robot0._actionsStack_.pop()
            else:
                break

        if(time.time() >= max_time):
            return False

        return True


    def compare_values_with_threshold(self, value1, value2):
        threshold = 0.8
        absolute_difference = abs(value1-value2)
        if absolute_difference < threshold:
            return True
        else:
            return False

    def print_function_name(self,name):
        print ""
        print "================================================="
        print name
        print "================================================="
