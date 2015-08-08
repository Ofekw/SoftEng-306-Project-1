#!/usr/bin/env python
#Adapted from http://sebsauvage.net/python/gui/#import

import sys
import threading
import Tkinter
import multiprocessing
import os
import atexit

"""
@author: Harry

GUI overlay class which contains information about each working robot on the orchard, including name,
type, position coordinates and also theta

"""
class GUI_overlay(Tkinter.Tk):




    def __init__(self):
        Tkinter.Tk.__init__(self)
        self.initialize()
        self.directory = "./"

    def initialize(self):
        self.minsize(width=400, height=500)

        self.grid()

        self.label_list = list()

        self.label0 = Tkinter.StringVar()
        label = Tkinter.Label(self,textvariable=self.label0,
                      anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=2,sticky='EW')

        self.label1 = Tkinter.StringVar()
        label1 = Tkinter.Label(self,textvariable=self.label1,
                      anchor="w",fg="black",bg="red")
        label1.grid(column=0,row=2,columnspan=2,sticky='EW')

        self.label2 = Tkinter.StringVar()
        label2 = Tkinter.Label(self,textvariable=self.label2,
                      anchor="w",fg="white",bg="green")
        label2.grid(column=0,row=3,columnspan=2,sticky='EW')

        self.label3 = Tkinter.StringVar()
        label3 = Tkinter.Label(self,textvariable=self.label3,
                      anchor="w",fg="black",bg="orange")
        label3.grid(column=0,row=4,columnspan=2,sticky='EW')

        self.label4 = Tkinter.StringVar()
        label4 = Tkinter.Label(self,textvariable=self.label4,
                      anchor="w",fg="white",bg="purple")
        label4.grid(column=0,row=5,columnspan=2,sticky='EW')


        self.label_list.append(self.label0)
        self.label_list.append(self.label1)
        self.label_list.append(self.label2)
        self.label_list.append(self.label3)
        self.label_list.append(self.label4)

        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False)
        self.geometry(self.geometry())
        self.title("Mission Control: Fl0PPY DI5KS")

    #Retrieve the new information every 50ms
    def update(self):
        i = 0;
        for file in os.listdir(self.directory):
            if file.endswith(".sta"):
                filename = file
                txt_again = open(filename)
                self.label_list[i].set(txt_again.read())
                i+=1
        self.after(50,self.update)

if __name__ == '__main__':
    gui = GUI_overlay()
    gui.after(0,gui.update)
    gui.mainloop()

def delete_files():
    for file in os.listdir("./"):
        if file.endswith(".sta"):
            print(str(file) + " Deleted" )
            os.remove(file)

atexit.register(delete_files())
