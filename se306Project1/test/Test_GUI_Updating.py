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

class Test_GUI_overlay_read_data(unittest.TestCase,TestModule):



    def test_gui_picker_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        gui = GUI_overlay()
        gui.directory = "./se306Project1/src/"
        file_path = './se306Project1/src/TESTpic.sta'
        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write("robot_100\n")
        output_file.write("Picker\n")
        output_file.write("Picking fruit\n")
        output_file.write("0.25\n")
        output_file.write("0.55\n")
        output_file.write("3.1415\n")
        output_file.write("19/20\n")
        output_file.close()

        gui.update()

        self.assertEquals(gui.robot_label_list[0].cget("text"), "robot_100\n")
        self.assertEquals(gui.robot_label_list[1].cget("text"), "Picker\n")
        self.assertEquals(gui.robot_label_list[2].cget("text"), "Picking fruit\n")
        self.assertEquals(gui.robot_label_list[3].cget("text"), "0.25\n")
        self.assertEquals(gui.robot_label_list[4].cget("text"), "0.55\n")
        self.assertEquals(gui.robot_label_list[5].cget("text"), "3.1415\n")
        self.assertEquals(gui.robot_label_list[6].cget("text"), "19/20\n")

        os.remove(file_path)

    def test_gui_carrier_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        gui = GUI_overlay()
        gui.directory = "./se306Project1/src/"
        file_path = './se306Project1/src/TESTcar.sta'
        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write("robot_200\n")
        output_file.write("Carrier\n")
        output_file.write("Going to Picker\n")
        output_file.write("0.25\n")
        output_file.write("0.55\n")
        output_file.write("3.1415\n")
        output_file.write("19/20\n")
        output_file.close()

        gui.update()

        self.assertEquals(gui.robot2_label_list[0].cget("text"), "robot_200\n")
        self.assertEquals(gui.robot2_label_list[1].cget("text"), "Carrier\n")
        self.assertEquals(gui.robot2_label_list[2].cget("text"), "Going to Picker\n")
        self.assertEquals(gui.robot2_label_list[3].cget("text"), "0.25\n")
        self.assertEquals(gui.robot2_label_list[4].cget("text"), "0.55\n")
        self.assertEquals(gui.robot2_label_list[5].cget("text"), "3.1415\n")
        self.assertEquals(gui.robot2_label_list[6].cget("text"), "19/20\n")

        os.remove(file_path)

    def test_gui_visitor_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        gui = GUI_overlay()
        gui.directory = "./se306Project1/src/"

        file_path = './se306Project1/src/TESTvis.sta'
        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write("human_500\n")
        output_file.write("Visitor\n")
        output_file.write("Doing random visitor stuff\n")
        output_file.write("0.25\n")
        output_file.write("0.55\n")
        output_file.write("3.1415\n")
        output_file.write("12/20\n")
        output_file.close()

        gui.update()

        self.assertEquals(gui.human_label_list[0].cget("text"), "human_500\n")
        self.assertEquals(gui.human_label_list[1].cget("text"), "Visitor\n")
        self.assertEquals(gui.human_label_list[2].cget("text"), "Doing random visitor stuff\n")
        self.assertEquals(gui.human_label_list[3].cget("text"), "0.25\n")
        self.assertEquals(gui.human_label_list[4].cget("text"), "0.55\n")
        self.assertEquals(gui.human_label_list[5].cget("text"), "3.1415\n")
        self.assertEquals(gui.human_label_list[6].cget("text"), "12/20\n")

        os.remove(file_path)

    def test_gui_worker_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        gui = GUI_overlay()
        gui.directory = "./se306Project1/src/"
        file_path = './se306Project1/src/TESTwor.sta'
        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write("human_500\n")
        output_file.write("Visitor\n")
        output_file.write("Doing random visitor stuff\n")
        output_file.write("0.25\n")
        output_file.write("0.55\n")
        output_file.write("3.1415\n")
        output_file.write("12/20\n")
        output_file.close()

        gui.update()

        self.assertEquals(gui.human_label_list[0].cget("text"), "human_500\n")
        self.assertEquals(gui.human_label_list[1].cget("text"), "Visitor\n")
        self.assertEquals(gui.human_label_list[2].cget("text"), "Doing random visitor stuff\n")
        self.assertEquals(gui.human_label_list[3].cget("text"), "0.25\n")
        self.assertEquals(gui.human_label_list[4].cget("text"), "0.55\n")
        self.assertEquals(gui.human_label_list[5].cget("text"), "3.1415\n")
        self.assertEquals(gui.human_label_list[6].cget("text"), "12/20\n")

        os.remove(file_path)

    def test_gui_animal_read(self):

        print self.print_function_name(inspect.stack()[0][3])

        gui = GUI_overlay()
        gui.directory = "./se306Project1/src/"

        file_path = './se306Project1/src/TESTani.sta'
        dir = os.path.abspath(file_path)
        output_file = open(dir, "w")
        output_file.write("human_500\n")
        output_file.write("Visitor\n")
        output_file.write("Doing random visitor stuff\n")
        output_file.write("0.25\n")
        output_file.write("0.55\n")
        output_file.write("3.1415\n")
        output_file.write("12/20\n")
        output_file.close()

        gui.update()

        self.assertEquals(gui.animal_label_list[0].cget("text"), "human_500\n")
        self.assertEquals(gui.animal_label_list[1].cget("text"), "Visitor\n")
        self.assertEquals(gui.animal_label_list[2].cget("text"), "Doing random visitor stuff\n")
        self.assertEquals(gui.animal_label_list[3].cget("text"), "0.25\n")
        self.assertEquals(gui.animal_label_list[4].cget("text"), "0.55\n")
        self.assertEquals(gui.animal_label_list[5].cget("text"), "3.1415\n")
        self.assertEquals(gui.animal_label_list[6].cget("text"), "12/20\n")

        os.remove(file_path)

if __name__ == '__main__':
    unittest.main()