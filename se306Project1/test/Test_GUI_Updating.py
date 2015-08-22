#!/usr/bin/env python
import unittest
from TestModule import TestModule
import rospy
from se306Project1.src.GUI_overlay import GUI_overlay
import inspect
import sys
import threading
import Tkinter
import multiprocessing
import os

class Test_GUI_Updating(unittest.TestCase,TestModule):

    def createFile(self,file_path,topic_name,entity_type,current_action,px,py,theta,capacity):

        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write(topic_name)
        output_file.write(entity_type)
        output_file.write(current_action)
        output_file.write(px)
        output_file.write(py)
        output_file.write(theta)
        output_file.write(capacity)
        output_file.close()

    def deleteFile(self,file_path):
        os.remove(file_path)

    def test_gui_picker_read(self):

        directory = "./se306Project1/src/"
        gui = GUI_overlay(directory)

        file_path = './se306Project1/src/0pic.sta'
        topic_name = "robot_100\n"
        entity_name = "Picker\n"
        current_action = "Picking fruit\n"
        px = "0.25\n"
        py = "0.55\n"
        theta = "3.1415\n"
        capacity = "19/25\n"

        self.createFile(file_path,topic_name,entity_name,current_action,px,py,theta,capacity)

        gui.update()

        self.deleteFile(file_path)

        self.assertEquals(gui.robot_label_list[0].cget("text"), topic_name)
        self.assertEquals(gui.robot_label_list[1].cget("text"), entity_name)
        self.assertEquals(gui.robot_label_list[2].cget("text"), current_action)
        self.assertEquals(gui.robot_label_list[3].cget("text"), px)
        self.assertEquals(gui.robot_label_list[4].cget("text"), py)
        self.assertEquals(gui.robot_label_list[5].cget("text"), theta)
        self.assertEquals(gui.robot_label_list[6].cget("text"), capacity)



    def test_gui_carrier_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        directory = "./se306Project1/src/"
        gui = GUI_overlay(directory)

        file_path = './se306Project1/src/0car.sta'
        topic_name = "robot_100\n"
        entity_name = "Picker\n"
        current_action = "Picking fruit\n"
        px = "0.25\n"
        py = "0.55\n"
        theta = "3.1415\n"
        capacity = "19/25\n"

        self.createFile(file_path,topic_name,entity_name,current_action,px,py,theta,capacity)

        gui.update()

        self.deleteFile(file_path)

        self.assertEquals(gui.robot2_label_list[0].cget("text"), topic_name)
        self.assertEquals(gui.robot2_label_list[1].cget("text"), entity_name)
        self.assertEquals(gui.robot2_label_list[2].cget("text"), current_action)
        self.assertEquals(gui.robot2_label_list[3].cget("text"), px)
        self.assertEquals(gui.robot2_label_list[4].cget("text"), py)
        self.assertEquals(gui.robot2_label_list[5].cget("text"), theta)
        self.assertEquals(gui.robot2_label_list[6].cget("text"), capacity)

    def test_gui_visitor_read(self):

        directory = "./se306Project1/src/"
        gui = GUI_overlay(directory)

        file_path = './se306Project1/src/0vis.sta'
        topic_name = "robot_100\n"
        entity_name = "Picker\n"
        current_action = "Picking fruit\n"
        px = "0.25\n"
        py = "0.55\n"
        theta = "3.1415\n"
        capacity = "19/25\n"

        self.createFile(file_path,topic_name,entity_name,current_action,px,py,theta,capacity)

        gui.update()

        self.deleteFile(file_path)

        self.assertEquals(gui.human_label_list[0].cget("text"), topic_name)
        self.assertEquals(gui.human_label_list[1].cget("text"), entity_name)
        self.assertEquals(gui.human_label_list[2].cget("text"), current_action)
        self.assertEquals(gui.human_label_list[3].cget("text"), px)
        self.assertEquals(gui.human_label_list[4].cget("text"), py)
        self.assertEquals(gui.human_label_list[5].cget("text"), theta)
        self.assertEquals(gui.human_label_list[6].cget("text"), capacity)


    def test_gui_worker_read(self):

        directory = "./se306Project1/src/"
        gui = GUI_overlay(directory)

        file_path = './se306Project1/src/0wor.sta'
        topic_name = "robot_100\n"
        entity_name = "Picker\n"
        current_action = "Picking fruit\n"
        px = "0.25\n"
        py = "0.55\n"
        theta = "3.1415\n"
        capacity = "19/25\n"

        self.createFile(file_path,topic_name,entity_name,current_action,px,py,theta,capacity)

        gui.update()

        self.deleteFile(file_path)

        self.assertEquals(gui.human_label_list[0].cget("text"), topic_name)
        self.assertEquals(gui.human_label_list[1].cget("text"), entity_name)
        self.assertEquals(gui.human_label_list[2].cget("text"), current_action)
        self.assertEquals(gui.human_label_list[3].cget("text"), px)
        self.assertEquals(gui.human_label_list[4].cget("text"), py)
        self.assertEquals(gui.human_label_list[5].cget("text"), theta)
        self.assertEquals(gui.human_label_list[6].cget("text"), capacity)

    def test_gui_animal_read(self):

        directory = "./se306Project1/src/"
        gui = GUI_overlay(directory)

        file_path = './se306Project1/src/0ani.sta'
        topic_name = "robot_100\n"
        entity_name = "Picker\n"
        current_action = "Picking fruit\n"
        px = "0.25\n"
        py = "0.55\n"
        theta = "3.1415\n"
        capacity = "19/25\n"

        self.createFile(file_path,topic_name,entity_name,current_action,px,py,theta,capacity)

        gui.update()

        self.deleteFile(file_path)

        self.assertEquals(gui.animal_label_list[0].cget("text"), topic_name)
        self.assertEquals(gui.animal_label_list[1].cget("text"), entity_name)
        self.assertEquals(gui.animal_label_list[2].cget("text"), current_action)
        self.assertEquals(gui.animal_label_list[3].cget("text"), px)
        self.assertEquals(gui.animal_label_list[4].cget("text"), py)
        self.assertEquals(gui.animal_label_list[5].cget("text"), theta)
        self.assertEquals(gui.animal_label_list[6].cget("text"), capacity)


if __name__ == '__main__':
    # unittest.main()
    import rostest
    rostest.rosrun('se306Project1', 'test_bare_bones', Test_GUI_Updating)