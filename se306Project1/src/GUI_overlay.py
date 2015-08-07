#!/usr/bin/env python
#Adapted from http://sebsauvage.net/python/gui/#import

import sys
from PyQt4 import QtGui
import threading
import Tkinter
import multiprocessing

"""
@author: Harry

GUI overlay class which contains information about each working robot on the orchard, including name,
type, position coordinates and also theta

"""
class GUI_overlay(Tkinter.Tk):

    def __init__(self):
        Tkinter.Tk.__init__(self)
        self.initialize()

    def initialize(self):
        self.minsize(width=400, height=500)

        self.grid()

        self.labelVariable = Tkinter.StringVar()
        label = Tkinter.Label(self,textvariable=self.labelVariable,
                      anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=2,sticky='EW')
        self.labelVariable.set(u"Hello !")

        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False)
        self.geometry(self.geometry())
        self.title("Mission Control: Fl0PPY DI5KS")

    #Retrieve the new information every 50ms
    def update(self):
        filename = "Picker0.txt"
        txt_again = open(filename)
        self.labelVariable.set(txt_again.read())
        self.after(50,self.update)


if __name__ == '__main__':
    gui = GUI_overlay()
    gui.after(0,gui.update)
    gui.mainloop()
