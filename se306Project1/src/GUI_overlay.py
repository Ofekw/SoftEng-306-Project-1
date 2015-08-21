#!/usr/bin/env python

import sys
import threading
import Tkinter
import multiprocessing
import os
import atexit
from matplotlib.figure import Figure
from numpy import arange, sin, pi
from Tkinter import *
import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import numpy as np
import matplotlib.pyplot as plt
import os
import string
import time


"""
@author: Harry

GUI overlay class which contains information about each working robot on the orchard, including name,
type, current action, current capacity (if applicable), position coordinates and also theta.

It also displays the laser ranger for picker robots in a seperate tab

"""
class GUI_overlay(Tkinter.Tk):

    def __init__(self,custom_dir="./"):
        Tkinter.Tk.__init__(self)
        #Custom _dir is for testing, it just allows a different directory to be passed in
        #if nothing is passed in... ie GUI_overlay(), it will just default to "./"
        self.directory = custom_dir
        self.initialize()



    def initialize(self):
        self.title('Mission Control: Fl0PPY D15K')
        geom = "700x850+603+196"
        self.geometry(geom)
        self.label_list = list()
        self.f = Figure(figsize=(5,4), dpi=100)


        self._bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        self._fgcolor = '#003399'  # X11 color: 'black'
        self._compcolor = '#d9d9d9' # X11 color: 'gray85'
        self._ana1color = '#d9d9d9' # X11 color: 'gray85'
        self._ana2color = '#d9d9d9' # X11 color: 'gray85'
        self.style = ttk.Style()
        if sys.platform == "win32":
            self.style.theme_use('winnative')
        self.style.configure('.',background=self._bgcolor)
        self.style.configure('.',foreground=self._fgcolor)
        self.style.configure('.',font="TkDefaultFont")
        self.style.map('.',background=
            [('selected', self._compcolor), ('active',self._ana2color)])
        self.configure(highlightcolor="black")

        nb = ttk.Notebook(self)
        nb.pack(fill='both', expand='yes')
        robot_tab = Frame(nb)
        robot_tab2 = Frame(nb)
        human_tab = Frame(nb)
        animal_tab = Frame(nb)
        self.laser_tab = Frame(nb)

        # create the tabs
        nb.add(robot_tab, text='Robot Pickers')
        nb.add(robot_tab2, text='Robot Carriers')
        nb.add(human_tab, text='Humans')
        nb.add(animal_tab, text='Animals')
        nb.add(self.laser_tab, text='Laser-Rangers')

        #BEGIN ROBOTS

        self.robot_label_list = self.setup_pickers(nb,robot_tab)
        self.robot2_label_list = self.setup_carriers(nb,robot_tab2)

        # BEGIN HUMANS ________________________________________________________
        self.human_label_list = self.setup_humans(nb,human_tab)

        # BEGIN ANIMALS _______________________________________________________-
        self.animal_label_list = self.setup_animals(nb,animal_tab)

        # BEGIN LASERS
        if self.directory == "./":
            #only sets up lasers if NOT in testing mode. (in testing mode, a custom directory is passed in.
            self.setup_lasers()




    def setup_pickers(self,nb,robot_tab):

        #ROBOT 1 ___________________________________________________________

        TLabelframe1 = ttk.Labelframe(robot_tab)
        TLabelframe1.place(relx=0.02, rely=0.03, relheight=0.17
                 , relwidth=0.95)
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(text='''Robot Report''')
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(width=390)

        robot_tab.Label1 = Label(TLabelframe1)
        robot_tab.Label1.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab.Label1.configure(text='''Name (ID):''')

        robot_tab.Label2 = Label(TLabelframe1)
        robot_tab.Label2.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab.Label2.configure(text='''X Position:''')

        robot_tab.Label3 = Label(TLabelframe1)
        robot_tab.Label3.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab.Label3.configure(text='''Y Position:''')

        robot_tab.Label4 = Label(TLabelframe1)
        robot_tab.Label4.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab.Label4.configure(text='''Theta (Rads):''')

        robot_tab.Label5 = Label(TLabelframe1)
        robot_tab.Label5.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab.Label5.configure(text='''Type:''')

        robot_tab.Label6 = Label(TLabelframe1)
        robot_tab.Label6.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab.Label6.configure(text='''Current action:''')

        robot_tab.Label7 = Label(TLabelframe1)
        robot_tab.Label7.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab.Label7.configure(text='''Current capacity:''')

        robot_tab.TLabel1 = ttk.Label(TLabelframe1)
        robot_tab.TLabel1.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab.TLabel1.configure(background=self._bgcolor)
        robot_tab.TLabel1.configure(foreground="#0066FF")
        robot_tab.TLabel1.configure(relief=FLAT)

        robot_tab.TLabel2 = ttk.Label(TLabelframe1)
        robot_tab.TLabel2.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab.TLabel2.configure(background=self._bgcolor)
        robot_tab.TLabel2.configure(foreground="#0066FF")
        robot_tab.TLabel2.configure(relief=FLAT)

        robot_tab.TLabel3 = ttk.Label(TLabelframe1)
        robot_tab.TLabel3.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab.TLabel3.configure(background=self._bgcolor)
        robot_tab.TLabel3.configure(foreground="#0066FF")
        robot_tab.TLabel3.configure(relief=FLAT)

        robot_tab.TLabel4 = ttk.Label(TLabelframe1)
        robot_tab.TLabel4.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab.TLabel4.configure(background=self._bgcolor)
        robot_tab.TLabel4.configure(foreground="#006600")
        robot_tab.TLabel4.configure(relief=FLAT)

        robot_tab.TLabel5 = ttk.Label(TLabelframe1)
        robot_tab.TLabel5.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab.TLabel5.configure(background=self._bgcolor)
        robot_tab.TLabel5.configure(foreground="#006600")
        robot_tab.TLabel5.configure(relief=FLAT)

        robot_tab.TLabel6 = ttk.Label(TLabelframe1)
        robot_tab.TLabel6.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab.TLabel6.configure(background=self._bgcolor)
        robot_tab.TLabel6.configure(foreground="#006600")
        robot_tab.TLabel6.configure(relief=FLAT)

        #ROBOT 2 ___________________________________________________________

        TLabelframe2 = ttk.Labelframe(robot_tab)
        TLabelframe2.place(relx=0.02, rely=0.22, relheight=0.17
                 , relwidth=0.95)
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(text='''Robot Report''')
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(width=390)

        robot_tab.Label8 = Label(TLabelframe2)
        robot_tab.Label8.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab.Label8.configure(activebackground="#f9f9f9")
        robot_tab.Label8.configure(text='''Name (ID):''')

        robot_tab.Label9 = Label(TLabelframe2)
        robot_tab.Label9.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab.Label9.configure(activebackground="#f9f9f9")
        robot_tab.Label9.configure(text='''X Position:''')

        robot_tab.Label10 = Label(TLabelframe2)
        robot_tab.Label10.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab.Label10.configure(activebackground="#f9f9f9")
        robot_tab.Label10.configure(text='''Y Position:''')

        robot_tab.Label11 = Label(TLabelframe2)
        robot_tab.Label11.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab.Label11.configure(activebackground="#f9f9f9")
        robot_tab.Label11.configure(text='''Theta (Rads):''')

        robot_tab.Label12 = Label(TLabelframe2)
        robot_tab.Label12.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab.Label12.configure(activebackground="#f9f9f9")
        robot_tab.Label12.configure(text='''Type:''')

        robot_tab.Label13 = Label(TLabelframe2)
        robot_tab.Label13.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab.Label13.configure(activebackground="#f9f9f9")
        robot_tab.Label13.configure(text='''Current action:''')

        robot_tab.Label14 = Label(TLabelframe2)
        robot_tab.Label14.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab.Label14.configure(activebackground="#f9f9f9")
        robot_tab.Label14.configure(text='''Current capacity:''')


        robot_tab.TLabel7 = ttk.Label(TLabelframe2)
        robot_tab.TLabel7.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab.TLabel7.configure(background=self._bgcolor)
        robot_tab.TLabel7.configure(foreground="#0066FF")
        robot_tab.TLabel7.configure(relief=FLAT)

        robot_tab.TLabel8 = ttk.Label(TLabelframe2)
        robot_tab.TLabel8.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab.TLabel8.configure(background=self._bgcolor)
        robot_tab.TLabel8.configure(foreground="#0066FF")
        robot_tab.TLabel8.configure(relief=FLAT)

        robot_tab.TLabel9 = ttk.Label(TLabelframe2)
        robot_tab.TLabel9.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab.TLabel9.configure(background=self._bgcolor)
        robot_tab.TLabel9.configure(foreground="#0066FF")
        robot_tab.TLabel9.configure(relief=FLAT)

        robot_tab.TLabel10 = ttk.Label(TLabelframe2)
        robot_tab.TLabel10.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab.TLabel10.configure(background=self._bgcolor)
        robot_tab.TLabel10.configure(foreground="#006600")
        robot_tab.TLabel10.configure(relief=FLAT)

        robot_tab.TLabel11 = ttk.Label(TLabelframe2)
        robot_tab.TLabel11.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab.TLabel11.configure(background=self._bgcolor)
        robot_tab.TLabel11.configure(foreground="#006600")
        robot_tab.TLabel11.configure(relief=FLAT)

        robot_tab.TLabel12 = ttk.Label(TLabelframe2)
        robot_tab.TLabel12.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab.TLabel12.configure(background=self._bgcolor)
        robot_tab.TLabel12.configure(foreground="#006600")
        robot_tab.TLabel12.configure(relief=FLAT)

        #ROBOT 3 ___________________________________________________________

        TLabelframe3 = ttk.Labelframe(robot_tab)
        TLabelframe3.place(relx=0.02, rely=0.41, relheight=0.17
                 , relwidth=0.95)
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(text="Robot Report")
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(width=390)

        robot_tab.Label15 = Label(TLabelframe3)
        robot_tab.Label15.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab.Label15.configure(activebackground="#f9f9f9")
        robot_tab.Label15.configure(text='''Name (ID):''')

        robot_tab.Label16 = Label(TLabelframe3)
        robot_tab.Label16.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab.Label16.configure(activebackground="#f9f9f9")
        robot_tab.Label16.configure(text='''X Position:''')

        robot_tab.Label17 = Label(TLabelframe3)
        robot_tab.Label17.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab.Label17.configure(activebackground="#f9f9f9")
        robot_tab.Label17.configure(text='''Y Position:''')

        robot_tab.Label18 = Label(TLabelframe3)
        robot_tab.Label18.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab.Label18.configure(activebackground="#f9f9f9")
        robot_tab.Label18.configure(text='''Theta (Rads):''')

        robot_tab.Label19 = Label(TLabelframe3)
        robot_tab.Label19.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab.Label19.configure(activebackground="#f9f9f9")
        robot_tab.Label19.configure(text='''Type:''')

        robot_tab.Label20 = Label(TLabelframe3)
        robot_tab.Label20.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab.Label20.configure(activebackground="#f9f9f9")
        robot_tab.Label20.configure(text='''Current action:''')

        robot_tab.Label21 = Label(TLabelframe3)
        robot_tab.Label21.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab.Label21.configure(activebackground="#f9f9f9")
        robot_tab.Label21.configure(text='''Current capacity:''')

        robot_tab.TLabel13 = ttk.Label(TLabelframe3)
        robot_tab.TLabel13.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab.TLabel13.configure(background=self._bgcolor)
        robot_tab.TLabel13.configure(foreground="#0066FF")
        robot_tab.TLabel13.configure(relief=FLAT)

        robot_tab.TLabel14 = ttk.Label(TLabelframe3)
        robot_tab.TLabel14.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab.TLabel14.configure(background=self._bgcolor)
        robot_tab.TLabel14.configure(foreground="#0066FF")
        robot_tab.TLabel14.configure(relief=FLAT)

        robot_tab.TLabel15 = ttk.Label(TLabelframe3)
        robot_tab.TLabel15.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab.TLabel15.configure(background=self._bgcolor)
        robot_tab.TLabel15.configure(foreground="#0066FF")
        robot_tab.TLabel15.configure(relief=FLAT)

        robot_tab.TLabel16 = ttk.Label(TLabelframe3)
        robot_tab.TLabel16.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab.TLabel16.configure(background=self._bgcolor)
        robot_tab.TLabel16.configure(foreground="#006600")
        robot_tab.TLabel16.configure(relief=FLAT)

        robot_tab.TLabel17 = ttk.Label(TLabelframe3)
        robot_tab.TLabel17.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab.TLabel17.configure(background=self._bgcolor)
        robot_tab.TLabel17.configure(foreground="#006600")
        robot_tab.TLabel17.configure(relief=FLAT)

        robot_tab.TLabel18 = ttk.Label(TLabelframe3)
        robot_tab.TLabel18.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab.TLabel18.configure(background=self._bgcolor)
        robot_tab.TLabel18.configure(foreground="#006600")
        robot_tab.TLabel18.configure(relief=FLAT)

        #ROBOT 4 ___________________________________________________________

        TLabelframe4 = ttk.Labelframe(robot_tab)
        TLabelframe4.place(relx=0.02, rely=0.6, relheight=0.17
                 , relwidth=0.95)
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(text='''Robot Report''')
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(width=390)

        robot_tab.Label22 = Label(TLabelframe4)
        robot_tab.Label22.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab.Label22.configure(activebackground="#f9f9f9")
        robot_tab.Label22.configure(text='''Name (ID):''')

        robot_tab.Label23 = Label(TLabelframe4)
        robot_tab.Label23.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab.Label23.configure(activebackground="#f9f9f9")
        robot_tab.Label23.configure(text='''X Position:''')

        robot_tab.Label24 = Label(TLabelframe4)
        robot_tab.Label24.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab.Label24.configure(activebackground="#f9f9f9")
        robot_tab.Label24.configure(text='''Y Position:''')

        robot_tab.Label25 = Label(TLabelframe4)
        robot_tab.Label25.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab.Label25.configure(activebackground="#f9f9f9")
        robot_tab.Label25.configure(text='''Theta (Rads):''')

        robot_tab.Label26 = Label(TLabelframe4)
        robot_tab.Label26.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab.Label26.configure(activebackground="#f9f9f9")
        robot_tab.Label26.configure(text='''Type:''')

        robot_tab.Label27 = Label(TLabelframe4)
        robot_tab.Label27.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab.Label27.configure(activebackground="#f9f9f9")
        robot_tab.Label27.configure(text='''Current action:''')

        robot_tab.Label28 = Label(TLabelframe4)
        robot_tab.Label28.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab.Label28.configure(activebackground="#f9f9f9")
        robot_tab.Label28.configure(text='''Current capacity:''')



        robot_tab.TLabel19 = ttk.Label(TLabelframe4)
        robot_tab.TLabel19.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab.TLabel19.configure(background=self._bgcolor)
        robot_tab.TLabel19.configure(foreground="#0066FF")
        robot_tab.TLabel19.configure(relief=FLAT)

        robot_tab.TLabel20 = ttk.Label(TLabelframe4)
        robot_tab.TLabel20.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab.TLabel20.configure(background=self._bgcolor)
        robot_tab.TLabel20.configure(foreground="#0066FF")
        robot_tab.TLabel20.configure(relief=FLAT)

        robot_tab.TLabel21 = ttk.Label(TLabelframe4)
        robot_tab.TLabel21.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab.TLabel21.configure(background=self._bgcolor)
        robot_tab.TLabel21.configure(foreground="#0066FF")
        robot_tab.TLabel21.configure(relief=FLAT)

        robot_tab.TLabel22 = ttk.Label(TLabelframe4)
        robot_tab.TLabel22.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab.TLabel22.configure(background=self._bgcolor)
        robot_tab.TLabel22.configure(foreground="#006600")
        robot_tab.TLabel22.configure(relief=FLAT)

        robot_tab.TLabel23 = ttk.Label(TLabelframe4)
        robot_tab.TLabel23.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab.TLabel23.configure(background=self._bgcolor)
        robot_tab.TLabel23.configure(foreground="#006600")
        robot_tab.TLabel23.configure(relief=FLAT)

        robot_tab.TLabel24 = ttk.Label(TLabelframe4)
        robot_tab.TLabel24.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab.TLabel24.configure(background=self._bgcolor)
        robot_tab.TLabel24.configure(foreground="#006600")
        robot_tab.TLabel24.configure(relief=FLAT)

        #ROBOT 5 ___________________________________________________________

        TLabelframe5 = ttk.Labelframe(robot_tab)
        TLabelframe5.place(relx=0.02, rely=0.79, relheight=0.17
                 , relwidth=0.95)
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(text='''Robot Report''')
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(width=390)

        robot_tab.Label29 = Label(TLabelframe5)
        robot_tab.Label29.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab.Label29.configure(activebackground="#f9f9f9")
        robot_tab.Label29.configure(text='''Name (ID):''')

        robot_tab.Label30 = Label(TLabelframe5)
        robot_tab.Label30.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab.Label30.configure(activebackground="#f9f9f9")
        robot_tab.Label30.configure(text='''X Position:''')

        robot_tab.Label31 = Label(TLabelframe5)
        robot_tab.Label31.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab.Label31.configure(activebackground="#f9f9f9")
        robot_tab.Label31.configure(text='''Y Position:''')

        robot_tab.Label32 = Label(TLabelframe5)
        robot_tab.Label32.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab.Label32.configure(activebackground="#f9f9f9")
        robot_tab.Label32.configure(text='''Theta (Rads):''')

        robot_tab.Label33 = Label(TLabelframe5)
        robot_tab.Label33.place(relx=0.03, rely=0.3, height=19, width=50)
        robot_tab.Label33.configure(activebackground="#f9f9f9")
        robot_tab.Label33.configure(text='''Type:''')

        robot_tab.Label34 = Label(TLabelframe5)
        robot_tab.Label34.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab.Label34.configure(activebackground="#f9f9f9")
        robot_tab.Label34.configure(text='''Current action:''')

        robot_tab.Label35 = Label(TLabelframe5)
        robot_tab.Label35.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab.Label35.configure(activebackground="#f9f9f9")
        robot_tab.Label35.configure(text='''Current capacity:''')


        robot_tab.TLabel25 = ttk.Label(TLabelframe5)
        robot_tab.TLabel25.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab.TLabel25.configure(background=self._bgcolor)
        robot_tab.TLabel25.configure(foreground="#0066FF")
        robot_tab.TLabel25.configure(relief=FLAT)

        robot_tab.TLabel26 = ttk.Label(TLabelframe5)
        robot_tab.TLabel26.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab.TLabel26.configure(background=self._bgcolor)
        robot_tab.TLabel26.configure(foreground="#0066FF")
        robot_tab.TLabel26.configure(relief=FLAT)

        robot_tab.TLabel27 = ttk.Label(TLabelframe5)
        robot_tab.TLabel27.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab.TLabel27.configure(background=self._bgcolor)
        robot_tab.TLabel27.configure(foreground="#0066FF")
        robot_tab.TLabel27.configure(relief=FLAT)

        robot_tab.TLabel28 = ttk.Label(TLabelframe5)
        robot_tab.TLabel28.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab.TLabel28.configure(background=self._bgcolor)
        robot_tab.TLabel28.configure(foreground="#006600")
        robot_tab.TLabel28.configure(relief=FLAT)

        robot_tab.TLabel29 = ttk.Label(TLabelframe5)
        robot_tab.TLabel29.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab.TLabel29.configure(background=self._bgcolor)
        robot_tab.TLabel29.configure(foreground="#006600")
        robot_tab.TLabel29.configure(relief=FLAT)

        robot_tab.TLabel30 = ttk.Label(TLabelframe5)
        robot_tab.TLabel30.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab.TLabel30.configure(background=self._bgcolor)
        robot_tab.TLabel30.configure(foreground="#006600")
        robot_tab.TLabel30.configure(relief=FLAT)

        robot_tab.Label36 = Label(TLabelframe1)
        robot_tab.Label36.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab.Label36.configure(cursor="fleur")
        robot_tab.Label36.configure(foreground="#FF0000")


        robot_tab.Label37 = Label(TLabelframe2)
        robot_tab.Label37.place(relx=0.31, rely=0.74, height=19, width=100)
        robot_tab.Label37.configure(width=56)
        robot_tab.Label37.configure(foreground="#FF0000")

        robot_tab.Label38 = Label(TLabelframe3)
        robot_tab.Label38.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab.Label38.configure(foreground="#FF0000")

        robot_tab.Label39 = Label(TLabelframe4)
        robot_tab.Label39.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab.Label39.configure(foreground="#FF0000")


        robot_tab.Label40 = Label(TLabelframe5)
        robot_tab.Label40.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab.Label40.configure(foreground="#FF0000")

        robot_label_list = [robot_tab.TLabel1,robot_tab.TLabel2,robot_tab.TLabel3,robot_tab.TLabel4,robot_tab.TLabel5,robot_tab.TLabel6,robot_tab.Label36, robot_tab.TLabel7,robot_tab.TLabel8,robot_tab.TLabel9,robot_tab.TLabel10,robot_tab.TLabel11,robot_tab.TLabel12,robot_tab.Label37,
                             robot_tab.TLabel13,robot_tab.TLabel14,robot_tab.TLabel15,robot_tab.TLabel16,robot_tab.TLabel17,robot_tab.TLabel18,robot_tab.Label38,robot_tab.TLabel19,robot_tab.TLabel20,robot_tab.TLabel21,robot_tab.TLabel22,robot_tab.TLabel23,robot_tab.TLabel24,robot_tab.Label39,
                             robot_tab.TLabel25,robot_tab.TLabel26,robot_tab.TLabel27,robot_tab.TLabel28,robot_tab.TLabel29,robot_tab.TLabel30,robot_tab.Label40]

        return robot_label_list

    def setup_carriers(self,nb,robot_tab2):

        #ROBOT 1 ___________________________________________________________

        TLabelframe1 = ttk.Labelframe(robot_tab2)
        TLabelframe1.place(relx=0.02, rely=0.03, relheight=0.17
                 , relwidth=0.95)
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(text='''Robot Report''')
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(width=390)

        robot_tab2.Label1 = Label(TLabelframe1)
        robot_tab2.Label1.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab2.Label1.configure(text='''Name (ID):''')

        robot_tab2.Label2 = Label(TLabelframe1)
        robot_tab2.Label2.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab2.Label2.configure(text='''X Position:''')

        robot_tab2.Label3 = Label(TLabelframe1)
        robot_tab2.Label3.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab2.Label3.configure(text='''Y Position:''')

        robot_tab2.Label4 = Label(TLabelframe1)
        robot_tab2.Label4.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab2.Label4.configure(text='''Theta (Rads):''')

        robot_tab2.Label5 = Label(TLabelframe1)
        robot_tab2.Label5.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab2.Label5.configure(text='''Type:''')

        robot_tab2.Label6 = Label(TLabelframe1)
        robot_tab2.Label6.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab2.Label6.configure(text='''Current action:''')

        robot_tab2.Label7 = Label(TLabelframe1)
        robot_tab2.Label7.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab2.Label7.configure(text='''Current capacity:''')

        robot_tab2.TLabel1 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel1.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab2.TLabel1.configure(background=self._bgcolor)
        robot_tab2.TLabel1.configure(foreground="#0066FF")
        robot_tab2.TLabel1.configure(relief=FLAT)

        robot_tab2.TLabel2 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel2.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab2.TLabel2.configure(background=self._bgcolor)
        robot_tab2.TLabel2.configure(foreground="#0066FF")
        robot_tab2.TLabel2.configure(relief=FLAT)

        robot_tab2.TLabel3 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel3.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab2.TLabel3.configure(background=self._bgcolor)
        robot_tab2.TLabel3.configure(foreground="#0066FF")
        robot_tab2.TLabel3.configure(relief=FLAT)

        robot_tab2.TLabel4 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel4.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab2.TLabel4.configure(background=self._bgcolor)
        robot_tab2.TLabel4.configure(foreground="#006600")
        robot_tab2.TLabel4.configure(relief=FLAT)

        robot_tab2.TLabel5 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel5.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab2.TLabel5.configure(background=self._bgcolor)
        robot_tab2.TLabel5.configure(foreground="#006600")
        robot_tab2.TLabel5.configure(relief=FLAT)

        robot_tab2.TLabel6 = ttk.Label(TLabelframe1)
        robot_tab2.TLabel6.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab2.TLabel6.configure(background=self._bgcolor)
        robot_tab2.TLabel6.configure(foreground="#006600")
        robot_tab2.TLabel6.configure(relief=FLAT)

        #ROBOT 2 ___________________________________________________________

        TLabelframe2 = ttk.Labelframe(robot_tab2)
        TLabelframe2.place(relx=0.02, rely=0.22, relheight=0.17
                 , relwidth=0.95)
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(text='''Robot Report''')
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(width=390)

        robot_tab2.Label8 = Label(TLabelframe2)
        robot_tab2.Label8.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab2.Label8.configure(activebackground="#f9f9f9")
        robot_tab2.Label8.configure(text='''Name (ID):''')

        robot_tab2.Label9 = Label(TLabelframe2)
        robot_tab2.Label9.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab2.Label9.configure(activebackground="#f9f9f9")
        robot_tab2.Label9.configure(text='''X Position:''')

        robot_tab2.Label10 = Label(TLabelframe2)
        robot_tab2.Label10.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab2.Label10.configure(activebackground="#f9f9f9")
        robot_tab2.Label10.configure(text='''Y Position:''')

        robot_tab2.Label11 = Label(TLabelframe2)
        robot_tab2.Label11.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab2.Label11.configure(activebackground="#f9f9f9")
        robot_tab2.Label11.configure(text='''Theta (Rads):''')

        robot_tab2.Label12 = Label(TLabelframe2)
        robot_tab2.Label12.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab2.Label12.configure(activebackground="#f9f9f9")
        robot_tab2.Label12.configure(text='''Type:''')

        robot_tab2.Label13 = Label(TLabelframe2)
        robot_tab2.Label13.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab2.Label13.configure(activebackground="#f9f9f9")
        robot_tab2.Label13.configure(text='''Current action:''')

        robot_tab2.Label14 = Label(TLabelframe2)
        robot_tab2.Label14.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab2.Label14.configure(activebackground="#f9f9f9")
        robot_tab2.Label14.configure(text='''Current capacity:''')


        robot_tab2.TLabel7 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel7.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab2.TLabel7.configure(background=self._bgcolor)
        robot_tab2.TLabel7.configure(foreground="#0066FF")
        robot_tab2.TLabel7.configure(relief=FLAT)

        robot_tab2.TLabel8 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel8.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab2.TLabel8.configure(background=self._bgcolor)
        robot_tab2.TLabel8.configure(foreground="#0066FF")
        robot_tab2.TLabel8.configure(relief=FLAT)

        robot_tab2.TLabel9 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel9.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab2.TLabel9.configure(background=self._bgcolor)
        robot_tab2.TLabel9.configure(foreground="#0066FF")
        robot_tab2.TLabel9.configure(relief=FLAT)

        robot_tab2.TLabel10 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel10.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab2.TLabel10.configure(background=self._bgcolor)
        robot_tab2.TLabel10.configure(foreground="#006600")
        robot_tab2.TLabel10.configure(relief=FLAT)

        robot_tab2.TLabel11 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel11.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab2.TLabel11.configure(background=self._bgcolor)
        robot_tab2.TLabel11.configure(foreground="#006600")
        robot_tab2.TLabel11.configure(relief=FLAT)

        robot_tab2.TLabel12 = ttk.Label(TLabelframe2)
        robot_tab2.TLabel12.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab2.TLabel12.configure(background=self._bgcolor)
        robot_tab2.TLabel12.configure(foreground="#006600")
        robot_tab2.TLabel12.configure(relief=FLAT)

        #ROBOT 3 ___________________________________________________________

        TLabelframe3 = ttk.Labelframe(robot_tab2)
        TLabelframe3.place(relx=0.02, rely=0.41, relheight=0.17
                 , relwidth=0.95)
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(text="Robot Report")
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(width=390)

        robot_tab2.Label15 = Label(TLabelframe3)
        robot_tab2.Label15.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab2.Label15.configure(activebackground="#f9f9f9")
        robot_tab2.Label15.configure(text='''Name (ID):''')

        robot_tab2.Label16 = Label(TLabelframe3)
        robot_tab2.Label16.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab2.Label16.configure(activebackground="#f9f9f9")
        robot_tab2.Label16.configure(text='''X Position:''')

        robot_tab2.Label17 = Label(TLabelframe3)
        robot_tab2.Label17.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab2.Label17.configure(activebackground="#f9f9f9")
        robot_tab2.Label17.configure(text='''Y Position:''')

        robot_tab2.Label18 = Label(TLabelframe3)
        robot_tab2.Label18.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab2.Label18.configure(activebackground="#f9f9f9")
        robot_tab2.Label18.configure(text='''Theta (Rads):''')

        robot_tab2.Label19 = Label(TLabelframe3)
        robot_tab2.Label19.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab2.Label19.configure(activebackground="#f9f9f9")
        robot_tab2.Label19.configure(text='''Type:''')

        robot_tab2.Label20 = Label(TLabelframe3)
        robot_tab2.Label20.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab2.Label20.configure(activebackground="#f9f9f9")
        robot_tab2.Label20.configure(text='''Current action:''')

        robot_tab2.Label21 = Label(TLabelframe3)
        robot_tab2.Label21.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab2.Label21.configure(activebackground="#f9f9f9")
        robot_tab2.Label21.configure(text='''Current capacity:''')

        robot_tab2.TLabel13 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel13.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab2.TLabel13.configure(background=self._bgcolor)
        robot_tab2.TLabel13.configure(foreground="#0066FF")
        robot_tab2.TLabel13.configure(relief=FLAT)

        robot_tab2.TLabel14 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel14.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab2.TLabel14.configure(background=self._bgcolor)
        robot_tab2.TLabel14.configure(foreground="#0066FF")
        robot_tab2.TLabel14.configure(relief=FLAT)

        robot_tab2.TLabel15 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel15.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab2.TLabel15.configure(background=self._bgcolor)
        robot_tab2.TLabel15.configure(foreground="#0066FF")
        robot_tab2.TLabel15.configure(relief=FLAT)

        robot_tab2.TLabel16 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel16.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab2.TLabel16.configure(background=self._bgcolor)
        robot_tab2.TLabel16.configure(foreground="#006600")
        robot_tab2.TLabel16.configure(relief=FLAT)

        robot_tab2.TLabel17 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel17.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab2.TLabel17.configure(background=self._bgcolor)
        robot_tab2.TLabel17.configure(foreground="#006600")
        robot_tab2.TLabel17.configure(relief=FLAT)

        robot_tab2.TLabel18 = ttk.Label(TLabelframe3)
        robot_tab2.TLabel18.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab2.TLabel18.configure(background=self._bgcolor)
        robot_tab2.TLabel18.configure(foreground="#006600")
        robot_tab2.TLabel18.configure(relief=FLAT)

        #ROBOT 4 ___________________________________________________________

        TLabelframe4 = ttk.Labelframe(robot_tab2)
        TLabelframe4.place(relx=0.02, rely=0.6, relheight=0.17
                 , relwidth=0.95)
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(text='''Robot Report''')
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(width=390)

        robot_tab2.Label22 = Label(TLabelframe4)
        robot_tab2.Label22.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab2.Label22.configure(activebackground="#f9f9f9")
        robot_tab2.Label22.configure(text='''Name (ID):''')

        robot_tab2.Label23 = Label(TLabelframe4)
        robot_tab2.Label23.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab2.Label23.configure(activebackground="#f9f9f9")
        robot_tab2.Label23.configure(text='''X Position:''')

        robot_tab2.Label24 = Label(TLabelframe4)
        robot_tab2.Label24.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab2.Label24.configure(activebackground="#f9f9f9")
        robot_tab2.Label24.configure(text='''Y Position:''')

        robot_tab2.Label25 = Label(TLabelframe4)
        robot_tab2.Label25.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab2.Label25.configure(activebackground="#f9f9f9")
        robot_tab2.Label25.configure(text='''Theta (Rads):''')

        robot_tab2.Label26 = Label(TLabelframe4)
        robot_tab2.Label26.place(relx=0.03, rely=0.3, height=19, width=37)
        robot_tab2.Label26.configure(activebackground="#f9f9f9")
        robot_tab2.Label26.configure(text='''Type:''')

        robot_tab2.Label27 = Label(TLabelframe4)
        robot_tab2.Label27.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab2.Label27.configure(activebackground="#f9f9f9")
        robot_tab2.Label27.configure(text='''Current action:''')

        robot_tab2.Label28 = Label(TLabelframe4)
        robot_tab2.Label28.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab2.Label28.configure(activebackground="#f9f9f9")
        robot_tab2.Label28.configure(text='''Current capacity:''')



        robot_tab2.TLabel19 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel19.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab2.TLabel19.configure(background=self._bgcolor)
        robot_tab2.TLabel19.configure(foreground="#0066FF")
        robot_tab2.TLabel19.configure(relief=FLAT)

        robot_tab2.TLabel20 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel20.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab2.TLabel20.configure(background=self._bgcolor)
        robot_tab2.TLabel20.configure(foreground="#0066FF")
        robot_tab2.TLabel20.configure(relief=FLAT)

        robot_tab2.TLabel21 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel21.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab2.TLabel21.configure(background=self._bgcolor)
        robot_tab2.TLabel21.configure(foreground="#0066FF")
        robot_tab2.TLabel21.configure(relief=FLAT)

        robot_tab2.TLabel22 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel22.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab2.TLabel22.configure(background=self._bgcolor)
        robot_tab2.TLabel22.configure(foreground="#006600")
        robot_tab2.TLabel22.configure(relief=FLAT)

        robot_tab2.TLabel23 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel23.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab2.TLabel23.configure(background=self._bgcolor)
        robot_tab2.TLabel23.configure(foreground="#006600")
        robot_tab2.TLabel23.configure(relief=FLAT)

        robot_tab2.TLabel24 = ttk.Label(TLabelframe4)
        robot_tab2.TLabel24.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab2.TLabel24.configure(background=self._bgcolor)
        robot_tab2.TLabel24.configure(foreground="#006600")
        robot_tab2.TLabel24.configure(relief=FLAT)

        #ROBOT 5 ___________________________________________________________

        TLabelframe5 = ttk.Labelframe(robot_tab2)
        TLabelframe5.place(relx=0.02, rely=0.79, relheight=0.17
                 , relwidth=0.95)
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(text='''Robot Report''')
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(width=390)

        robot_tab2.Label29 = Label(TLabelframe5)
        robot_tab2.Label29.place(relx=0.025, rely=0.15, height=19, width=74)
        robot_tab2.Label29.configure(activebackground="#f9f9f9")
        robot_tab2.Label29.configure(text='''Name (ID):''')

        robot_tab2.Label30 = Label(TLabelframe5)
        robot_tab2.Label30.place(relx=0.54, rely=0.07, height=19, width=66)
        robot_tab2.Label30.configure(activebackground="#f9f9f9")
        robot_tab2.Label30.configure(text='''X Position:''')

        robot_tab2.Label31 = Label(TLabelframe5)
        robot_tab2.Label31.place(relx=0.54, rely=0.22, height=19, width=65)
        robot_tab2.Label31.configure(activebackground="#f9f9f9")
        robot_tab2.Label31.configure(text='''Y Position:''')

        robot_tab2.Label32 = Label(TLabelframe5)
        robot_tab2.Label32.place(relx=0.54, rely=0.37, height=19, width=85)
        robot_tab2.Label32.configure(activebackground="#f9f9f9")
        robot_tab2.Label32.configure(text='''Theta (Rads):''')

        robot_tab2.Label33 = Label(TLabelframe5)
        robot_tab2.Label33.place(relx=0.03, rely=0.3, height=19, width=50)
        robot_tab2.Label33.configure(activebackground="#f9f9f9")
        robot_tab2.Label33.configure(text='''Type:''')

        robot_tab2.Label34 = Label(TLabelframe5)
        robot_tab2.Label34.place(relx=0.03, rely=0.52, height=19, width=95)
        robot_tab2.Label34.configure(activebackground="#f9f9f9")
        robot_tab2.Label34.configure(text='''Current action:''')

        robot_tab2.Label35 = Label(TLabelframe5)
        robot_tab2.Label35.place(relx=0.03, rely=0.74, height=19, width=109)
        robot_tab2.Label35.configure(activebackground="#f9f9f9")
        robot_tab2.Label35.configure(text='''Current capacity:''')


        robot_tab2.TLabel25 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel25.place(relx=0.26, rely=0.15, height=17, width=50)
        robot_tab2.TLabel25.configure(background=self._bgcolor)
        robot_tab2.TLabel25.configure(foreground="#0066FF")
        robot_tab2.TLabel25.configure(relief=FLAT)

        robot_tab2.TLabel26 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel26.place(relx=0.26, rely=0.3, height=17, width=50)
        robot_tab2.TLabel26.configure(background=self._bgcolor)
        robot_tab2.TLabel26.configure(foreground="#0066FF")
        robot_tab2.TLabel26.configure(relief=FLAT)

        robot_tab2.TLabel27 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel27.place(relx=0.31, rely=0.52, height=17, width=200)
        robot_tab2.TLabel27.configure(background=self._bgcolor)
        robot_tab2.TLabel27.configure(foreground="#0066FF")
        robot_tab2.TLabel27.configure(relief=FLAT)

        robot_tab2.TLabel28 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel28.place(relx=0.79, rely=0.07, height=17, width=50)
        robot_tab2.TLabel28.configure(background=self._bgcolor)
        robot_tab2.TLabel28.configure(foreground="#006600")
        robot_tab2.TLabel28.configure(relief=FLAT)

        robot_tab2.TLabel29 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel29.place(relx=0.79, rely=0.22, height=17, width=50)
        robot_tab2.TLabel29.configure(background=self._bgcolor)
        robot_tab2.TLabel29.configure(foreground="#006600")
        robot_tab2.TLabel29.configure(relief=FLAT)

        robot_tab2.TLabel30 = ttk.Label(TLabelframe5)
        robot_tab2.TLabel30.place(relx=0.79, rely=0.37, height=17, width=50)
        robot_tab2.TLabel30.configure(background=self._bgcolor)
        robot_tab2.TLabel30.configure(foreground="#006600")
        robot_tab2.TLabel30.configure(relief=FLAT)

        robot_tab2.Label36 = Label(TLabelframe1)
        robot_tab2.Label36.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab2.Label36.configure(cursor="fleur")
        robot_tab2.Label36.configure(foreground="#FF0000")


        robot_tab2.Label37 = Label(TLabelframe2)
        robot_tab2.Label37.place(relx=0.31, rely=0.74, height=19, width=100)
        robot_tab2.Label37.configure(width=56)
        robot_tab2.Label37.configure(foreground="#FF0000")

        robot_tab2.Label38 = Label(TLabelframe3)
        robot_tab2.Label38.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab2.Label38.configure(foreground="#FF0000")

        robot_tab2.Label39 = Label(TLabelframe4)
        robot_tab2.Label39.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab2.Label39.configure(foreground="#FF0000")


        robot_tab2.Label40 = Label(TLabelframe5)
        robot_tab2.Label40.place(relx=0.33, rely=0.74, height=19, width=100)
        robot_tab2.Label40.configure(foreground="#FF0000")

        robot2_label_list = [robot_tab2.TLabel1,robot_tab2.TLabel2,robot_tab2.TLabel3,robot_tab2.TLabel4,robot_tab2.TLabel5,robot_tab2.TLabel6,robot_tab2.Label36, robot_tab2.TLabel7,robot_tab2.TLabel8,robot_tab2.TLabel9,robot_tab2.TLabel10,robot_tab2.TLabel11,robot_tab2.TLabel12,robot_tab2.Label37,
                             robot_tab2.TLabel13,robot_tab2.TLabel14,robot_tab2.TLabel15,robot_tab2.TLabel16,robot_tab2.TLabel17,robot_tab2.TLabel18,robot_tab2.Label38,robot_tab2.TLabel19,robot_tab2.TLabel20,robot_tab2.TLabel21,robot_tab2.TLabel22,robot_tab2.TLabel23,robot_tab2.TLabel24,robot_tab2.Label39,
                             robot_tab2.TLabel25,robot_tab2.TLabel26,robot_tab2.TLabel27,robot_tab2.TLabel28,robot_tab2.TLabel29,robot_tab2.TLabel30,robot_tab2.Label40]

        return robot2_label_list

    def setup_humans(self,nb,human_tab):

        #HUMAN 1 ___________________________________________________________

        TLabelframe1 = ttk.Labelframe(human_tab)
        TLabelframe1.place(relx=0.02, rely=0.03, relheight=0.17
                 , relwidth=0.95)
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(text='''Human Report''')
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(width=390)

        human_tab.Label1 = Label(TLabelframe1)
        human_tab.Label1.place(relx=0.025, rely=0.15, height=19, width=74)
        human_tab.Label1.configure(text='''Name (ID):''')

        human_tab.Label2 = Label(TLabelframe1)
        human_tab.Label2.place(relx=0.54, rely=0.07, height=19, width=66)
        human_tab.Label2.configure(text='''X Position:''')

        human_tab.Label3 = Label(TLabelframe1)
        human_tab.Label3.place(relx=0.54, rely=0.22, height=19, width=65)
        human_tab.Label3.configure(text='''Y Position:''')

        human_tab.Label4 = Label(TLabelframe1)
        human_tab.Label4.place(relx=0.54, rely=0.37, height=19, width=85)
        human_tab.Label4.configure(text='''Theta (Rads):''')

        human_tab.Label5 = Label(TLabelframe1)
        human_tab.Label5.place(relx=0.03, rely=0.3, height=19, width=37)
        human_tab.Label5.configure(text='''Type:''')

        human_tab.Label6 = Label(TLabelframe1)
        human_tab.Label6.place(relx=0.03, rely=0.6, height=19, width=95)
        human_tab.Label6.configure(text='''Current action:''')

        human_tab.TLabel1 = ttk.Label(TLabelframe1)
        human_tab.TLabel1.place(relx=0.26, rely=0.15, height=17, width=75)
        human_tab.TLabel1.configure(background=self._bgcolor)
        human_tab.TLabel1.configure(foreground="#0066FF")
        human_tab.TLabel1.configure(relief=FLAT)

        human_tab.TLabel2 = ttk.Label(TLabelframe1)
        human_tab.TLabel2.place(relx=0.26, rely=0.3, height=17, width=50)
        human_tab.TLabel2.configure(background=self._bgcolor)
        human_tab.TLabel2.configure(foreground="#0066FF")
        human_tab.TLabel2.configure(relief=FLAT)

        human_tab.TLabel3 = ttk.Label(TLabelframe1)
        human_tab.TLabel3.place(relx=0.31, rely=0.6, height=17, width=200)
        human_tab.TLabel3.configure(background=self._bgcolor)
        human_tab.TLabel3.configure(foreground="#0066FF")
        human_tab.TLabel3.configure(relief=FLAT)

        human_tab.TLabel4 = ttk.Label(TLabelframe1)
        human_tab.TLabel4.place(relx=0.79, rely=0.07, height=17, width=50)
        human_tab.TLabel4.configure(background=self._bgcolor)
        human_tab.TLabel4.configure(foreground="#006600")
        human_tab.TLabel4.configure(relief=FLAT)

        human_tab.TLabel5 = ttk.Label(TLabelframe1)
        human_tab.TLabel5.place(relx=0.79, rely=0.22, height=17, width=50)
        human_tab.TLabel5.configure(background=self._bgcolor)
        human_tab.TLabel5.configure(foreground="#006600")
        human_tab.TLabel5.configure(relief=FLAT)

        human_tab.TLabel6 = ttk.Label(TLabelframe1)
        human_tab.TLabel6.place(relx=0.79, rely=0.37, height=17, width=50)
        human_tab.TLabel6.configure(background=self._bgcolor)
        human_tab.TLabel6.configure(foreground="#006600")
        human_tab.TLabel6.configure(relief=FLAT)


        #HUMAN 2 ___________________________________________________________

        TLabelframe2 = ttk.Labelframe(human_tab)
        TLabelframe2.place(relx=0.02, rely=0.22, relheight=0.17
                 , relwidth=0.95)
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(text='''Human Report''')
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(width=390)

        human_tab.Label8 = Label(TLabelframe2)
        human_tab.Label8.place(relx=0.025, rely=0.15, height=19, width=74)
        human_tab.Label8.configure(activebackground="#f9f9f9")
        human_tab.Label8.configure(text='''Name (ID):''')

        human_tab.Label9 = Label(TLabelframe2)
        human_tab.Label9.place(relx=0.54, rely=0.07, height=19, width=66)
        human_tab.Label9.configure(activebackground="#f9f9f9")
        human_tab.Label9.configure(text='''X Position:''')

        human_tab.Label10 = Label(TLabelframe2)
        human_tab.Label10.place(relx=0.54, rely=0.22, height=19, width=65)
        human_tab.Label10.configure(activebackground="#f9f9f9")
        human_tab.Label10.configure(text='''Y Position:''')

        human_tab.Label11 = Label(TLabelframe2)
        human_tab.Label11.place(relx=0.54, rely=0.37, height=19, width=85)
        human_tab.Label11.configure(activebackground="#f9f9f9")
        human_tab.Label11.configure(text='''Theta (Rads):''')

        human_tab.Label12 = Label(TLabelframe2)
        human_tab.Label12.place(relx=0.03, rely=0.3, height=19, width=37)
        human_tab.Label12.configure(activebackground="#f9f9f9")
        human_tab.Label12.configure(text='''Type:''')

        human_tab.Label13 = Label(TLabelframe2)
        human_tab.Label13.place(relx=0.03, rely=0.6, height=19, width=95)
        human_tab.Label13.configure(activebackground="#f9f9f9")
        human_tab.Label13.configure(text='''Current action:''')

        human_tab.TLabel7 = ttk.Label(TLabelframe2)
        human_tab.TLabel7.place(relx=0.26, rely=0.15, height=17, width=75)
        human_tab.TLabel7.configure(background=self._bgcolor)
        human_tab.TLabel7.configure(foreground="#0066FF")
        human_tab.TLabel7.configure(relief=FLAT)

        human_tab.TLabel8 = ttk.Label(TLabelframe2)
        human_tab.TLabel8.place(relx=0.26, rely=0.3, height=17, width=50)
        human_tab.TLabel8.configure(background=self._bgcolor)
        human_tab.TLabel8.configure(foreground="#0066FF")
        human_tab.TLabel8.configure(relief=FLAT)

        human_tab.TLabel9 = ttk.Label(TLabelframe2)
        human_tab.TLabel9.place(relx=0.31, rely=0.6, height=17, width=200)
        human_tab.TLabel9.configure(background=self._bgcolor)
        human_tab.TLabel9.configure(foreground="#0066FF")
        human_tab.TLabel9.configure(relief=FLAT)

        human_tab.TLabel10 = ttk.Label(TLabelframe2)
        human_tab.TLabel10.place(relx=0.79, rely=0.07, height=17, width=50)
        human_tab.TLabel10.configure(background=self._bgcolor)
        human_tab.TLabel10.configure(foreground="#006600")
        human_tab.TLabel10.configure(relief=FLAT)

        human_tab.TLabel11 = ttk.Label(TLabelframe2)
        human_tab.TLabel11.place(relx=0.79, rely=0.22, height=17, width=50)
        human_tab.TLabel11.configure(background=self._bgcolor)
        human_tab.TLabel11.configure(foreground="#006600")
        human_tab.TLabel11.configure(relief=FLAT)

        human_tab.TLabel12 = ttk.Label(TLabelframe2)
        human_tab.TLabel12.place(relx=0.79, rely=0.37, height=17, width=50)
        human_tab.TLabel12.configure(background=self._bgcolor)
        human_tab.TLabel12.configure(foreground="#006600")
        human_tab.TLabel12.configure(relief=FLAT)


#HUMAN 3 ________________________________________________________

        TLabelframe3 = ttk.Labelframe(human_tab)
        TLabelframe3.place(relx=0.02, rely=0.41, relheight=0.17
                 , relwidth=0.95)
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(text="Human Report")
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(width=390)

        human_tab.Label15 = Label(TLabelframe3)
        human_tab.Label15.place(relx=0.025, rely=0.15, height=19, width=74)
        human_tab.Label15.configure(activebackground="#f9f9f9")
        human_tab.Label15.configure(text='''Name (ID):''')

        human_tab.Label16 = Label(TLabelframe3)
        human_tab.Label16.place(relx=0.54, rely=0.07, height=19, width=66)
        human_tab.Label16.configure(activebackground="#f9f9f9")
        human_tab.Label16.configure(text='''X Position:''')

        human_tab.Label17 = Label(TLabelframe3)
        human_tab.Label17.place(relx=0.54, rely=0.22, height=19, width=65)
        human_tab.Label17.configure(activebackground="#f9f9f9")
        human_tab.Label17.configure(text='''Y Position:''')

        human_tab.Label18 = Label(TLabelframe3)
        human_tab.Label18.place(relx=0.54, rely=0.37, height=19, width=85)
        human_tab.Label18.configure(activebackground="#f9f9f9")
        human_tab.Label18.configure(text='''Theta (Rads):''')

        human_tab.Label19 = Label(TLabelframe3)
        human_tab.Label19.place(relx=0.03, rely=0.3, height=19, width=37)
        human_tab.Label19.configure(activebackground="#f9f9f9")
        human_tab.Label19.configure(text='''Type:''')

        human_tab.Label20 = Label(TLabelframe3)
        human_tab.Label20.place(relx=0.03, rely=0.52, height=19, width=95)
        human_tab.Label20.configure(activebackground="#f9f9f9")
        human_tab.Label20.configure(text='''Current action:''')

        human_tab.TLabel13 = ttk.Label(TLabelframe3)
        human_tab.TLabel13.place(relx=0.26, rely=0.15, height=17, width=50)
        human_tab.TLabel13.configure(background=self._bgcolor)
        human_tab.TLabel13.configure(foreground="#0066FF")
        human_tab.TLabel13.configure(relief=FLAT)

        human_tab.TLabel14 = ttk.Label(TLabelframe3)
        human_tab.TLabel14.place(relx=0.26, rely=0.3, height=17, width=50)
        human_tab.TLabel14.configure(background=self._bgcolor)
        human_tab.TLabel14.configure(foreground="#0066FF")
        human_tab.TLabel14.configure(relief=FLAT)

        human_tab.TLabel15 = ttk.Label(TLabelframe3)
        human_tab.TLabel15.place(relx=0.31, rely=0.52, height=17, width=200)
        human_tab.TLabel15.configure(background=self._bgcolor)
        human_tab.TLabel15.configure(foreground="#0066FF")
        human_tab.TLabel15.configure(relief=FLAT)

        human_tab.TLabel16 = ttk.Label(TLabelframe3)
        human_tab.TLabel16.place(relx=0.79, rely=0.07, height=17, width=50)
        human_tab.TLabel16.configure(background=self._bgcolor)
        human_tab.TLabel16.configure(foreground="#006600")
        human_tab.TLabel16.configure(relief=FLAT)

        human_tab.TLabel17 = ttk.Label(TLabelframe3)
        human_tab.TLabel17.place(relx=0.79, rely=0.22, height=17, width=50)
        human_tab.TLabel17.configure(background=self._bgcolor)
        human_tab.TLabel17.configure(foreground="#006600")
        human_tab.TLabel17.configure(relief=FLAT)

        human_tab.TLabel18 = ttk.Label(TLabelframe3)
        human_tab.TLabel18.place(relx=0.79, rely=0.37, height=17, width=50)
        human_tab.TLabel18.configure(background=self._bgcolor)
        human_tab.TLabel18.configure(foreground="#006600")
        human_tab.TLabel18.configure(relief=FLAT)

        #HUMAN 4 ___________________________________________________________

        TLabelframe4 = ttk.Labelframe(human_tab)
        TLabelframe4.place(relx=0.02, rely=0.6, relheight=0.17
                 , relwidth=0.95)
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(text='''Human Report''')
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(width=390)

        human_tab.Label22 = Label(TLabelframe4)
        human_tab.Label22.place(relx=0.025, rely=0.15, height=19, width=74)
        human_tab.Label22.configure(activebackground="#f9f9f9")
        human_tab.Label22.configure(text='''Name (ID):''')

        human_tab.Label23 = Label(TLabelframe4)
        human_tab.Label23.place(relx=0.54, rely=0.07, height=19, width=66)
        human_tab.Label23.configure(activebackground="#f9f9f9")
        human_tab.Label23.configure(text='''X Position:''')

        human_tab.Label24 = Label(TLabelframe4)
        human_tab.Label24.place(relx=0.54, rely=0.22, height=19, width=65)
        human_tab.Label24.configure(activebackground="#f9f9f9")
        human_tab.Label24.configure(text='''Y Position:''')

        human_tab.Label25 = Label(TLabelframe4)
        human_tab.Label25.place(relx=0.54, rely=0.37, height=19, width=85)
        human_tab.Label25.configure(activebackground="#f9f9f9")
        human_tab.Label25.configure(text='''Theta (Rads):''')

        human_tab.Label26 = Label(TLabelframe4)
        human_tab.Label26.place(relx=0.03, rely=0.3, height=19, width=37)
        human_tab.Label26.configure(activebackground="#f9f9f9")
        human_tab.Label26.configure(text='''Type:''')

        human_tab.Label27 = Label(TLabelframe4)
        human_tab.Label27.place(relx=0.03, rely=0.52, height=19, width=95)
        human_tab.Label27.configure(activebackground="#f9f9f9")
        human_tab.Label27.configure(text='''Current action:''')

        human_tab.TLabel19 = ttk.Label(TLabelframe4)
        human_tab.TLabel19.place(relx=0.26, rely=0.15, height=17, width=50)
        human_tab.TLabel19.configure(background=self._bgcolor)
        human_tab.TLabel19.configure(foreground="#0066FF")
        human_tab.TLabel19.configure(relief=FLAT)

        human_tab.TLabel20 = ttk.Label(TLabelframe4)
        human_tab.TLabel20.place(relx=0.26, rely=0.3, height=17, width=50)
        human_tab.TLabel20.configure(background=self._bgcolor)
        human_tab.TLabel20.configure(foreground="#0066FF")
        human_tab.TLabel20.configure(relief=FLAT)

        human_tab.TLabel21 = ttk.Label(TLabelframe4)
        human_tab.TLabel21.place(relx=0.31, rely=0.52, height=17, width=200)
        human_tab.TLabel21.configure(background=self._bgcolor)
        human_tab.TLabel21.configure(foreground="#0066FF")
        human_tab.TLabel21.configure(relief=FLAT)

        human_tab.TLabel22 = ttk.Label(TLabelframe4)
        human_tab.TLabel22.place(relx=0.79, rely=0.07, height=17, width=50)
        human_tab.TLabel22.configure(background=self._bgcolor)
        human_tab.TLabel22.configure(foreground="#006600")
        human_tab.TLabel22.configure(relief=FLAT)

        human_tab.TLabel23 = ttk.Label(TLabelframe4)
        human_tab.TLabel23.place(relx=0.79, rely=0.22, height=17, width=50)
        human_tab.TLabel23.configure(background=self._bgcolor)
        human_tab.TLabel23.configure(foreground="#006600")
        human_tab.TLabel23.configure(relief=FLAT)

        human_tab.TLabel24 = ttk.Label(TLabelframe4)
        human_tab.TLabel24.place(relx=0.79, rely=0.37, height=17, width=50)
        human_tab.TLabel24.configure(background=self._bgcolor)
        human_tab.TLabel24.configure(foreground="#006600")
        human_tab.TLabel24.configure(relief=FLAT)

        #HUMAN 5 ___________________________________________________________

        TLabelframe5 = ttk.Labelframe(human_tab)
        TLabelframe5.place(relx=0.02, rely=0.79, relheight=0.17
                 , relwidth=0.95)
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(text='''Human Report''')
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(width=390)

        human_tab.Label29 = Label(TLabelframe5)
        human_tab.Label29.place(relx=0.025, rely=0.15, height=19, width=74)
        human_tab.Label29.configure(activebackground="#f9f9f9")
        human_tab.Label29.configure(text='''Name (ID):''')

        human_tab.Label30 = Label(TLabelframe5)
        human_tab.Label30.place(relx=0.54, rely=0.07, height=19, width=66)
        human_tab.Label30.configure(activebackground="#f9f9f9")
        human_tab.Label30.configure(text='''X Position:''')

        human_tab.Label31 = Label(TLabelframe5)
        human_tab.Label31.place(relx=0.54, rely=0.22, height=19, width=65)
        human_tab.Label31.configure(activebackground="#f9f9f9")
        human_tab.Label31.configure(text='''Y Position:''')

        human_tab.Label32 = Label(TLabelframe5)
        human_tab.Label32.place(relx=0.54, rely=0.37, height=19, width=85)
        human_tab.Label32.configure(activebackground="#f9f9f9")
        human_tab.Label32.configure(text='''Theta (Rads):''')

        human_tab.Label33 = Label(TLabelframe5)
        human_tab.Label33.place(relx=0.03, rely=0.3, height=19, width=50)
        human_tab.Label33.configure(activebackground="#f9f9f9")
        human_tab.Label33.configure(text='''Type:''')

        human_tab.Label34 = Label(TLabelframe5)
        human_tab.Label34.place(relx=0.03, rely=0.52, height=19, width=95)
        human_tab.Label34.configure(activebackground="#f9f9f9")
        human_tab.Label34.configure(text='''Current action:''')


        human_tab.TLabel25 = ttk.Label(TLabelframe5)
        human_tab.TLabel25.place(relx=0.26, rely=0.15, height=17, width=50)
        human_tab.TLabel25.configure(background=self._bgcolor)
        human_tab.TLabel25.configure(foreground="#0066FF")
        human_tab.TLabel25.configure(relief=FLAT)

        human_tab.TLabel26 = ttk.Label(TLabelframe5)
        human_tab.TLabel26.place(relx=0.26, rely=0.3, height=17, width=50)
        human_tab.TLabel26.configure(background=self._bgcolor)
        human_tab.TLabel26.configure(foreground="#0066FF")
        human_tab.TLabel26.configure(relief=FLAT)

        human_tab.TLabel27 = ttk.Label(TLabelframe5)
        human_tab.TLabel27.place(relx=0.31, rely=0.52, height=17, width=200)
        human_tab.TLabel27.configure(background=self._bgcolor)
        human_tab.TLabel27.configure(foreground="#0066FF")
        human_tab.TLabel27.configure(relief=FLAT)

        human_tab.TLabel28 = ttk.Label(TLabelframe5)
        human_tab.TLabel28.place(relx=0.79, rely=0.07, height=17, width=50)
        human_tab.TLabel28.configure(background=self._bgcolor)
        human_tab.TLabel28.configure(foreground="#006600")
        human_tab.TLabel28.configure(relief=FLAT)

        human_tab.TLabel29 = ttk.Label(TLabelframe5)
        human_tab.TLabel29.place(relx=0.79, rely=0.22, height=17, width=50)
        human_tab.TLabel29.configure(background=self._bgcolor)
        human_tab.TLabel29.configure(foreground="#006600")
        human_tab.TLabel29.configure(relief=FLAT)

        human_tab.TLabel30 = ttk.Label(TLabelframe5)
        human_tab.TLabel30.place(relx=0.79, rely=0.37, height=17, width=50)
        human_tab.TLabel30.configure(background=self._bgcolor)
        human_tab.TLabel30.configure(foreground="#006600")
        human_tab.TLabel30.configure(relief=FLAT)


        human_label_list = [human_tab.TLabel1,human_tab.TLabel2,human_tab.TLabel3,human_tab.TLabel4,human_tab.TLabel5,human_tab.TLabel6, human_tab.TLabel7,human_tab.TLabel8,human_tab.TLabel9,human_tab.TLabel10,human_tab.TLabel11,human_tab.TLabel12,
                             human_tab.TLabel13,human_tab.TLabel14,human_tab.TLabel15,human_tab.TLabel16,human_tab.TLabel17,human_tab.TLabel18,human_tab.TLabel19,human_tab.TLabel20,human_tab.TLabel21,human_tab.TLabel22,human_tab.TLabel23,human_tab.TLabel24,
                             human_tab.TLabel25,human_tab.TLabel26,human_tab.TLabel27,human_tab.TLabel28,human_tab.TLabel29,human_tab.TLabel30]

        return human_label_list

    def setup_animals(self,nb,animal_tab):

               #ANIMAL 1 ___________________________________________________________

        TLabelframe1 = ttk.Labelframe(animal_tab)
        TLabelframe1.place(relx=0.02, rely=0.03, relheight=0.17
                 , relwidth=0.95)
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(text='''Animal Report''')
        TLabelframe1.configure(relief=RAISED)
        TLabelframe1.configure(width=390)

        animal_tab.Label1 = Label(TLabelframe1)
        animal_tab.Label1.place(relx=0.025, rely=0.15, height=19, width=74)
        animal_tab.Label1.configure(text='''Name (ID):''')

        animal_tab.Label2 = Label(TLabelframe1)
        animal_tab.Label2.place(relx=0.54, rely=0.07, height=19, width=66)
        animal_tab.Label2.configure(text='''X Position:''')

        animal_tab.Label3 = Label(TLabelframe1)
        animal_tab.Label3.place(relx=0.54, rely=0.22, height=19, width=65)
        animal_tab.Label3.configure(text='''Y Position:''')

        animal_tab.Label4 = Label(TLabelframe1)
        animal_tab.Label4.place(relx=0.54, rely=0.37, height=19, width=85)
        animal_tab.Label4.configure(text='''Theta (Rads):''')

        animal_tab.Label5 = Label(TLabelframe1)
        animal_tab.Label5.place(relx=0.03, rely=0.3, height=19, width=37)
        animal_tab.Label5.configure(text='''Type:''')

        animal_tab.Label6 = Label(TLabelframe1)
        animal_tab.Label6.place(relx=0.03, rely=0.6, height=19, width=95)
        animal_tab.Label6.configure(text='''Current action:''')

        animal_tab.TLabel1 = ttk.Label(TLabelframe1)
        animal_tab.TLabel1.place(relx=0.26, rely=0.15, height=17, width=75)
        animal_tab.TLabel1.configure(background=self._bgcolor)
        animal_tab.TLabel1.configure(foreground="#0066FF")
        animal_tab.TLabel1.configure(relief=FLAT)

        animal_tab.TLabel2 = ttk.Label(TLabelframe1)
        animal_tab.TLabel2.place(relx=0.26, rely=0.3, height=17, width=50)
        animal_tab.TLabel2.configure(background=self._bgcolor)
        animal_tab.TLabel2.configure(foreground="#0066FF")
        animal_tab.TLabel2.configure(relief=FLAT)

        animal_tab.TLabel3 = ttk.Label(TLabelframe1)
        animal_tab.TLabel3.place(relx=0.31, rely=0.6, height=17, width=200)
        animal_tab.TLabel3.configure(background=self._bgcolor)
        animal_tab.TLabel3.configure(foreground="#0066FF")
        animal_tab.TLabel3.configure(relief=FLAT)

        animal_tab.TLabel4 = ttk.Label(TLabelframe1)
        animal_tab.TLabel4.place(relx=0.79, rely=0.07, height=17, width=50)
        animal_tab.TLabel4.configure(background=self._bgcolor)
        animal_tab.TLabel4.configure(foreground="#006600")
        animal_tab.TLabel4.configure(relief=FLAT)

        animal_tab.TLabel5 = ttk.Label(TLabelframe1)
        animal_tab.TLabel5.place(relx=0.79, rely=0.22, height=17, width=50)
        animal_tab.TLabel5.configure(background=self._bgcolor)
        animal_tab.TLabel5.configure(foreground="#006600")
        animal_tab.TLabel5.configure(relief=FLAT)

        animal_tab.TLabel6 = ttk.Label(TLabelframe1)
        animal_tab.TLabel6.place(relx=0.79, rely=0.37, height=17, width=50)
        animal_tab.TLabel6.configure(background=self._bgcolor)
        animal_tab.TLabel6.configure(foreground="#006600")
        animal_tab.TLabel6.configure(relief=FLAT)


        #ANIMAL 2 ___________________________________________________________

        TLabelframe2 = ttk.Labelframe(animal_tab)
        TLabelframe2.place(relx=0.02, rely=0.22, relheight=0.17
                 , relwidth=0.95)
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(text='''Animal Report''')
        TLabelframe2.configure(relief=RAISED)
        TLabelframe2.configure(width=390)

        animal_tab.Label8 = Label(TLabelframe2)
        animal_tab.Label8.place(relx=0.025, rely=0.15, height=19, width=74)
        animal_tab.Label8.configure(activebackground="#f9f9f9")
        animal_tab.Label8.configure(text='''Name (ID):''')

        animal_tab.Label9 = Label(TLabelframe2)
        animal_tab.Label9.place(relx=0.54, rely=0.07, height=19, width=66)
        animal_tab.Label9.configure(activebackground="#f9f9f9")
        animal_tab.Label9.configure(text='''X Position:''')

        animal_tab.Label10 = Label(TLabelframe2)
        animal_tab.Label10.place(relx=0.54, rely=0.22, height=19, width=65)
        animal_tab.Label10.configure(activebackground="#f9f9f9")
        animal_tab.Label10.configure(text='''Y Position:''')

        animal_tab.Label11 = Label(TLabelframe2)
        animal_tab.Label11.place(relx=0.54, rely=0.37, height=19, width=85)
        animal_tab.Label11.configure(activebackground="#f9f9f9")
        animal_tab.Label11.configure(text='''Theta (Rads):''')

        animal_tab.Label12 = Label(TLabelframe2)
        animal_tab.Label12.place(relx=0.03, rely=0.3, height=19, width=37)
        animal_tab.Label12.configure(activebackground="#f9f9f9")
        animal_tab.Label12.configure(text='''Type:''')

        animal_tab.Label13 = Label(TLabelframe2)
        animal_tab.Label13.place(relx=0.03, rely=0.6, height=19, width=95)
        animal_tab.Label13.configure(activebackground="#f9f9f9")
        animal_tab.Label13.configure(text='''Current action:''')

        animal_tab.TLabel7 = ttk.Label(TLabelframe2)
        animal_tab.TLabel7.place(relx=0.26, rely=0.15, height=17, width=75)
        animal_tab.TLabel7.configure(background=self._bgcolor)
        animal_tab.TLabel7.configure(foreground="#0066FF")
        animal_tab.TLabel7.configure(relief=FLAT)

        animal_tab.TLabel8 = ttk.Label(TLabelframe2)
        animal_tab.TLabel8.place(relx=0.26, rely=0.3, height=17, width=50)
        animal_tab.TLabel8.configure(background=self._bgcolor)
        animal_tab.TLabel8.configure(foreground="#0066FF")
        animal_tab.TLabel8.configure(relief=FLAT)

        animal_tab.TLabel9 = ttk.Label(TLabelframe2)
        animal_tab.TLabel9.place(relx=0.31, rely=0.6, height=17, width=200)
        animal_tab.TLabel9.configure(background=self._bgcolor)
        animal_tab.TLabel9.configure(foreground="#0066FF")
        animal_tab.TLabel9.configure(relief=FLAT)

        animal_tab.TLabel10 = ttk.Label(TLabelframe2)
        animal_tab.TLabel10.place(relx=0.79, rely=0.07, height=17, width=50)
        animal_tab.TLabel10.configure(background=self._bgcolor)
        animal_tab.TLabel10.configure(foreground="#006600")
        animal_tab.TLabel10.configure(relief=FLAT)

        animal_tab.TLabel11 = ttk.Label(TLabelframe2)
        animal_tab.TLabel11.place(relx=0.79, rely=0.22, height=17, width=50)
        animal_tab.TLabel11.configure(background=self._bgcolor)
        animal_tab.TLabel11.configure(foreground="#006600")
        animal_tab.TLabel11.configure(relief=FLAT)

        animal_tab.TLabel12 = ttk.Label(TLabelframe2)
        animal_tab.TLabel12.place(relx=0.79, rely=0.37, height=17, width=50)
        animal_tab.TLabel12.configure(background=self._bgcolor)
        animal_tab.TLabel12.configure(foreground="#006600")
        animal_tab.TLabel12.configure(relief=FLAT)


#ANIMAL 3 ________________________________________________________

        TLabelframe3 = ttk.Labelframe(animal_tab)
        TLabelframe3.place(relx=0.02, rely=0.41, relheight=0.17
                 , relwidth=0.95)
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(text="Animal Report")
        TLabelframe3.configure(relief=RAISED)
        TLabelframe3.configure(width=390)

        animal_tab.Label15 = Label(TLabelframe3)
        animal_tab.Label15.place(relx=0.025, rely=0.15, height=19, width=74)
        animal_tab.Label15.configure(activebackground="#f9f9f9")
        animal_tab.Label15.configure(text='''Name (ID):''')

        animal_tab.Label16 = Label(TLabelframe3)
        animal_tab.Label16.place(relx=0.54, rely=0.07, height=19, width=66)
        animal_tab.Label16.configure(activebackground="#f9f9f9")
        animal_tab.Label16.configure(text='''X Position:''')

        animal_tab.Label17 = Label(TLabelframe3)
        animal_tab.Label17.place(relx=0.54, rely=0.22, height=19, width=65)
        animal_tab.Label17.configure(activebackground="#f9f9f9")
        animal_tab.Label17.configure(text='''Y Position:''')

        animal_tab.Label18 = Label(TLabelframe3)
        animal_tab.Label18.place(relx=0.54, rely=0.37, height=19, width=85)
        animal_tab.Label18.configure(activebackground="#f9f9f9")
        animal_tab.Label18.configure(text='''Theta (Rads):''')

        animal_tab.Label19 = Label(TLabelframe3)
        animal_tab.Label19.place(relx=0.03, rely=0.3, height=19, width=37)
        animal_tab.Label19.configure(activebackground="#f9f9f9")
        animal_tab.Label19.configure(text='''Type:''')

        animal_tab.Label20 = Label(TLabelframe3)
        animal_tab.Label20.place(relx=0.03, rely=0.52, height=19, width=95)
        animal_tab.Label20.configure(activebackground="#f9f9f9")
        animal_tab.Label20.configure(text='''Current action:''')

        animal_tab.TLabel13 = ttk.Label(TLabelframe3)
        animal_tab.TLabel13.place(relx=0.26, rely=0.15, height=17, width=50)
        animal_tab.TLabel13.configure(background=self._bgcolor)
        animal_tab.TLabel13.configure(foreground="#0066FF")
        animal_tab.TLabel13.configure(relief=FLAT)

        animal_tab.TLabel14 = ttk.Label(TLabelframe3)
        animal_tab.TLabel14.place(relx=0.26, rely=0.3, height=17, width=50)
        animal_tab.TLabel14.configure(background=self._bgcolor)
        animal_tab.TLabel14.configure(foreground="#0066FF")
        animal_tab.TLabel14.configure(relief=FLAT)

        animal_tab.TLabel15 = ttk.Label(TLabelframe3)
        animal_tab.TLabel15.place(relx=0.31, rely=0.52, height=17, width=200)
        animal_tab.TLabel15.configure(background=self._bgcolor)
        animal_tab.TLabel15.configure(foreground="#0066FF")
        animal_tab.TLabel15.configure(relief=FLAT)

        animal_tab.TLabel16 = ttk.Label(TLabelframe3)
        animal_tab.TLabel16.place(relx=0.79, rely=0.07, height=17, width=50)
        animal_tab.TLabel16.configure(background=self._bgcolor)
        animal_tab.TLabel16.configure(foreground="#006600")
        animal_tab.TLabel16.configure(relief=FLAT)

        animal_tab.TLabel17 = ttk.Label(TLabelframe3)
        animal_tab.TLabel17.place(relx=0.79, rely=0.22, height=17, width=50)
        animal_tab.TLabel17.configure(background=self._bgcolor)
        animal_tab.TLabel17.configure(foreground="#006600")
        animal_tab.TLabel17.configure(relief=FLAT)

        animal_tab.TLabel18 = ttk.Label(TLabelframe3)
        animal_tab.TLabel18.place(relx=0.79, rely=0.37, height=17, width=50)
        animal_tab.TLabel18.configure(background=self._bgcolor)
        animal_tab.TLabel18.configure(foreground="#006600")
        animal_tab.TLabel18.configure(relief=FLAT)

        #ANIMAL 4 ___________________________________________________________

        TLabelframe4 = ttk.Labelframe(animal_tab)
        TLabelframe4.place(relx=0.02, rely=0.6, relheight=0.17
                 , relwidth=0.95)
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(text='''Animal Report''')
        TLabelframe4.configure(relief=RAISED)
        TLabelframe4.configure(width=390)

        animal_tab.Label22 = Label(TLabelframe4)
        animal_tab.Label22.place(relx=0.025, rely=0.15, height=19, width=74)
        animal_tab.Label22.configure(activebackground="#f9f9f9")
        animal_tab.Label22.configure(text='''Name (ID):''')

        animal_tab.Label23 = Label(TLabelframe4)
        animal_tab.Label23.place(relx=0.54, rely=0.07, height=19, width=66)
        animal_tab.Label23.configure(activebackground="#f9f9f9")
        animal_tab.Label23.configure(text='''X Position:''')

        animal_tab.Label24 = Label(TLabelframe4)
        animal_tab.Label24.place(relx=0.54, rely=0.22, height=19, width=65)
        animal_tab.Label24.configure(activebackground="#f9f9f9")
        animal_tab.Label24.configure(text='''Y Position:''')

        animal_tab.Label25 = Label(TLabelframe4)
        animal_tab.Label25.place(relx=0.54, rely=0.37, height=19, width=85)
        animal_tab.Label25.configure(activebackground="#f9f9f9")
        animal_tab.Label25.configure(text='''Theta (Rads):''')

        animal_tab.Label26 = Label(TLabelframe4)
        animal_tab.Label26.place(relx=0.03, rely=0.3, height=19, width=37)
        animal_tab.Label26.configure(activebackground="#f9f9f9")
        animal_tab.Label26.configure(text='''Type:''')

        animal_tab.Label27 = Label(TLabelframe4)
        animal_tab.Label27.place(relx=0.03, rely=0.52, height=19, width=95)
        animal_tab.Label27.configure(activebackground="#f9f9f9")
        animal_tab.Label27.configure(text='''Current action:''')

        animal_tab.TLabel19 = ttk.Label(TLabelframe4)
        animal_tab.TLabel19.place(relx=0.26, rely=0.15, height=17, width=50)
        animal_tab.TLabel19.configure(background=self._bgcolor)
        animal_tab.TLabel19.configure(foreground="#0066FF")
        animal_tab.TLabel19.configure(relief=FLAT)

        animal_tab.TLabel20 = ttk.Label(TLabelframe4)
        animal_tab.TLabel20.place(relx=0.26, rely=0.3, height=17, width=50)
        animal_tab.TLabel20.configure(background=self._bgcolor)
        animal_tab.TLabel20.configure(foreground="#0066FF")
        animal_tab.TLabel20.configure(relief=FLAT)

        animal_tab.TLabel21 = ttk.Label(TLabelframe4)
        animal_tab.TLabel21.place(relx=0.31, rely=0.52, height=17, width=200)
        animal_tab.TLabel21.configure(background=self._bgcolor)
        animal_tab.TLabel21.configure(foreground="#0066FF")
        animal_tab.TLabel21.configure(relief=FLAT)

        animal_tab.TLabel22 = ttk.Label(TLabelframe4)
        animal_tab.TLabel22.place(relx=0.79, rely=0.07, height=17, width=50)
        animal_tab.TLabel22.configure(background=self._bgcolor)
        animal_tab.TLabel22.configure(foreground="#006600")
        animal_tab.TLabel22.configure(relief=FLAT)

        animal_tab.TLabel23 = ttk.Label(TLabelframe4)
        animal_tab.TLabel23.place(relx=0.79, rely=0.22, height=17, width=50)
        animal_tab.TLabel23.configure(background=self._bgcolor)
        animal_tab.TLabel23.configure(foreground="#006600")
        animal_tab.TLabel23.configure(relief=FLAT)

        animal_tab.TLabel24 = ttk.Label(TLabelframe4)
        animal_tab.TLabel24.place(relx=0.79, rely=0.37, height=17, width=50)
        animal_tab.TLabel24.configure(background=self._bgcolor)
        animal_tab.TLabel24.configure(foreground="#006600")
        animal_tab.TLabel24.configure(relief=FLAT)

        #ANIMAL 5 ___________________________________________________________

        TLabelframe5 = ttk.Labelframe(animal_tab)
        TLabelframe5.place(relx=0.02, rely=0.79, relheight=0.17
                 , relwidth=0.95)
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(text='''Animal Report''')
        TLabelframe5.configure(relief=RAISED)
        TLabelframe5.configure(width=390)

        animal_tab.Label29 = Label(TLabelframe5)
        animal_tab.Label29.place(relx=0.025, rely=0.15, height=19, width=74)
        animal_tab.Label29.configure(activebackground="#f9f9f9")
        animal_tab.Label29.configure(text='''Name (ID):''')

        animal_tab.Label30 = Label(TLabelframe5)
        animal_tab.Label30.place(relx=0.54, rely=0.07, height=19, width=66)
        animal_tab.Label30.configure(activebackground="#f9f9f9")
        animal_tab.Label30.configure(text='''X Position:''')

        animal_tab.Label31 = Label(TLabelframe5)
        animal_tab.Label31.place(relx=0.54, rely=0.22, height=19, width=65)
        animal_tab.Label31.configure(activebackground="#f9f9f9")
        animal_tab.Label31.configure(text='''Y Position:''')

        animal_tab.Label32 = Label(TLabelframe5)
        animal_tab.Label32.place(relx=0.54, rely=0.37, height=19, width=85)
        animal_tab.Label32.configure(activebackground="#f9f9f9")
        animal_tab.Label32.configure(text='''Theta (Rads):''')

        animal_tab.Label33 = Label(TLabelframe5)
        animal_tab.Label33.place(relx=0.03, rely=0.3, height=19, width=50)
        animal_tab.Label33.configure(activebackground="#f9f9f9")
        animal_tab.Label33.configure(text='''Type:''')

        animal_tab.Label34 = Label(TLabelframe5)
        animal_tab.Label34.place(relx=0.03, rely=0.52, height=19, width=95)
        animal_tab.Label34.configure(activebackground="#f9f9f9")
        animal_tab.Label34.configure(text='''Current action:''')

        animal_tab.TLabel25 = ttk.Label(TLabelframe5)
        animal_tab.TLabel25.place(relx=0.26, rely=0.15, height=17, width=50)
        animal_tab.TLabel25.configure(background=self._bgcolor)
        animal_tab.TLabel25.configure(foreground="#0066FF")
        animal_tab.TLabel25.configure(relief=FLAT)

        animal_tab.TLabel26 = ttk.Label(TLabelframe5)
        animal_tab.TLabel26.place(relx=0.26, rely=0.3, height=17, width=50)
        animal_tab.TLabel26.configure(background=self._bgcolor)
        animal_tab.TLabel26.configure(foreground="#0066FF")
        animal_tab.TLabel26.configure(relief=FLAT)

        animal_tab.TLabel27 = ttk.Label(TLabelframe5)
        animal_tab.TLabel27.place(relx=0.31, rely=0.52, height=17, width=200)
        animal_tab.TLabel27.configure(background=self._bgcolor)
        animal_tab.TLabel27.configure(foreground="#0066FF")
        animal_tab.TLabel27.configure(relief=FLAT)

        animal_tab.TLabel28 = ttk.Label(TLabelframe5)
        animal_tab.TLabel28.place(relx=0.79, rely=0.07, height=17, width=50)
        animal_tab.TLabel28.configure(background=self._bgcolor)
        animal_tab.TLabel28.configure(foreground="#006600")
        animal_tab.TLabel28.configure(relief=FLAT)

        animal_tab.TLabel29 = ttk.Label(TLabelframe5)
        animal_tab.TLabel29.place(relx=0.79, rely=0.22, height=17, width=50)
        animal_tab.TLabel29.configure(background=self._bgcolor)
        animal_tab.TLabel29.configure(foreground="#006600")
        animal_tab.TLabel29.configure(relief=FLAT)

        animal_tab.TLabel30 = ttk.Label(TLabelframe5)
        animal_tab.TLabel30.place(relx=0.79, rely=0.37, height=17, width=50)
        animal_tab.TLabel30.configure(background=self._bgcolor)
        animal_tab.TLabel30.configure(foreground="#006600")
        animal_tab.TLabel30.configure(relief=FLAT)



        animal_label_list = [animal_tab.TLabel1,animal_tab.TLabel2,animal_tab.TLabel3,animal_tab.TLabel4,animal_tab.TLabel5,animal_tab.TLabel6, animal_tab.TLabel7,animal_tab.TLabel8,animal_tab.TLabel9,animal_tab.TLabel10,animal_tab.TLabel11,animal_tab.TLabel12,
                             animal_tab.TLabel13,animal_tab.TLabel14,animal_tab.TLabel15,animal_tab.TLabel16,animal_tab.TLabel17,animal_tab.TLabel18,animal_tab.TLabel19,animal_tab.TLabel20,animal_tab.TLabel21,animal_tab.TLabel22,animal_tab.TLabel23,animal_tab.TLabel24,
                             animal_tab.TLabel25,animal_tab.TLabel26,animal_tab.TLabel27,animal_tab.TLabel28,animal_tab.TLabel29,animal_tab.TLabel30]
        return animal_label_list

    #Setup those laser rangers
    def setup_lasers(self):

        self.LaserFrame1 = ttk.Labelframe(self.laser_tab)
        self.LaserFrame1.place(relx=0.02, rely=0.03, relheight=0.46
                 , relwidth=0.46)
        self.LaserFrame1.configure(relief=RAISED)
        self.LaserFrame1.configure(text='''Lasers''')
        self.LaserFrame1.configure(relief=RAISED)
        self.LaserFrame1.configure(width=390)

        self.LaserFrame2 = ttk.Labelframe(self.laser_tab)
        self.LaserFrame2.place(relx=0.52, rely=0.03, relheight=0.46
                 , relwidth=0.46)
        self.LaserFrame2.configure(relief=RAISED)
        self.LaserFrame2.configure(text='''Lasers''')
        self.LaserFrame2.configure(relief=RAISED)
        self.LaserFrame2.configure(width=390)

        self.LaserFrame3 = ttk.Labelframe(self.laser_tab)
        self.LaserFrame3.place(relx=0.02, rely=0.51, relheight=0.46
                 , relwidth=0.46)
        self.LaserFrame3.configure(relief=RAISED)
        self.LaserFrame3.configure(text='''Lasers''')
        self.LaserFrame3.configure(relief=RAISED)
        self.LaserFrame3.configure(width=390)

        self.LaserFrame4 = ttk.Labelframe(self.laser_tab)
        self.LaserFrame4.place(relx=0.52, rely=0.51, relheight=0.46
                 , relwidth=0.46)
        self.LaserFrame4.configure(relief=RAISED)
        self.LaserFrame4.configure(text='''Lasers''')
        self.LaserFrame4.configure(relief=RAISED)
        self.LaserFrame4.configure(width=390)


        directory = "./"
        count = 0
        read = False
        while read == False:
            for file in os.listdir(directory):
                if (file.endswith("laser.ls")) and (count == 0):
                    read = True
                    time.sleep(3)
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    N = 180
                    self.theta = np.linspace(0.0, np.pi, N, endpoint=False)
                    radii = floats
                    self.width = np.pi / 180
                    self.f1 = Figure(figsize=(5,4), dpi=100)
                    self.a1 = self.f1.add_subplot(111, polar=True)
                    if (len(radii)==180):
                        self.bars1 = self.a1.bar(self.theta, radii, color='g',width=self.width, bottom=0.0,edgecolor='none' )
                        self.canvas1 = FigureCanvasTkAgg(self.f1, master=self.LaserFrame1)
                        self.canvas1.show()
                        self.canvas1.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
                elif (file.endswith("laser.ls")) and (count == 1):
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    N = 180
                    self.theta = np.linspace(0.0, np.pi, N, endpoint=False)
                    radii = floats
                    self.width = np.pi / 180
                    self.f2 = Figure(figsize=(5,4), dpi=100)
                    self.a2 = self.f2.add_subplot(111, polar=True)
                    if (len(radii)==180):
                        self.bars2 = self.a2.bar(self.theta, radii, color='g',width=self.width, bottom=0.0,edgecolor='none' )
                        self.canvas2 = FigureCanvasTkAgg(self.f2, master=self.LaserFrame2)
                        self.canvas2.show()
                        self.canvas2.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
                elif (file.endswith("laser.ls")) and (count == 2):
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    N = 180
                    self.theta = np.linspace(0.0, np.pi, N, endpoint=False)
                    radii =  floats
                    self.width = np.pi / 180
                    self.f3 = Figure(figsize=(5,4), dpi=100)
                    self.a3 = self.f3.add_subplot(111, polar=True)
                    if (len(radii)==180):
                        self.bars3 = self.a3.bar(self.theta, radii, color='g',width=self.width, bottom=0.0,edgecolor='none' )
                        self.canvas3 = FigureCanvasTkAgg(self.f3, master=self.LaserFrame3)
                        self.canvas3.show()
                        self.canvas3.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
                elif (file.endswith("laser.ls")) and (count == 3):
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    N = 180
                    self.theta = np.linspace(0.0, np.pi, N, endpoint=False)
                    radii =  floats
                    self.width = np.pi / 180
                    self.f4 = Figure(figsize=(5,4), dpi=100)
                    self.a4 = self.f4.add_subplot(111, polar=True)
                    if (len(radii)==180):
                        self.bars4 = self.a4.bar(self.theta, radii, color='g',width=self.width, bottom=0.0,edgecolor='none' )
                        self.canvas4 = FigureCanvasTkAgg(self.f4, master=self.LaserFrame4)
                        self.canvas4.show()
                        self.canvas4.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)


    """
    @function

    Function called by thread to update laser information by reading in the data from the pickers
    """
    def update_lasers(self):
        while True:
            directory = "./"
            count=0
            for file in os.listdir(directory):
                if (file.endswith("laser.ls")) and (count == 0):
                    try:
                        self.LaserFrame1.configure(text="Robot Picker "+str(file[0]))
                    except RuntimeError:
                        pass
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    r =  floats
                    if (len(r)==180):
                        for bar, radius in zip(self.bars1, r):
                            bar.set_height(radius)
                        try:
                            self.canvas1.draw()
                        except RuntimeError:
                            pass
                elif (file.endswith("laser.ls")) and (count == 1):
                    try:
                        self.LaserFrame2.configure(text="Robot Picker "+str(file[0]))
                    except RuntimeError:
                        pass
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    r =  floats
                    if (len(r)==180):
                        for bar, radius in zip(self.bars2, r):
                            bar.set_height(radius)
                        try:
                            self.canvas2.draw()
                        except RuntimeError:
                            pass
                elif (file.endswith("laser.ls")) and (count == 2):
                    try:
                        self.LaserFrame3.configure(text="Robot Picker "+str(file[0]))
                    except RuntimeError:
                        pass
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    r =  floats
                    if (len(r)==180):
                        for bar, radius in zip(self.bars3, r):
                            bar.set_height(radius)
                        try:
                            self.canvas3.draw()
                        except RuntimeError:
                            pass
                elif (file.endswith("laser.ls")) and (count == 3):
                    try:
                        self.LaserFrame4.configure(text="Robot Picker "+str(file[0]))
                    except RuntimeError:
                        pass
                    count+=1
                    f = open(file)
                    lines = f.read()
                    lines = lines.translate(None, '\',[]')
                    floats = [float(x) for x in lines.split()]
                    r =  floats
                    if (len(r)==180):
                        for bar, radius in zip(self.bars4, r):
                            bar.set_height(radius)
                        try:
                            self.canvas4.draw()
                        except RuntimeError:
                            pass




    """
    @function

    Main GUI updater that grabs the Entity states and displays them every few milliseconds

    """
    def update(self):
        i = 0
        for file in os.listdir(self.directory):
            if file.endswith("pic.sta"):
                with open(self.directory+file) as f:
                    for line in f:
                        self.robot_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        for file in os.listdir(self.directory):
            if file.endswith("car.sta"):
                with open(self.directory+file) as f:
                    for line in f:
                        self.robot2_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        for file in os.listdir(self.directory):
            if file.endswith("vis.sta") or file.endswith("wor.sta"):
                with open(self.directory+file) as f:
                    for line in f:
                        self.human_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        for file in os.listdir(self.directory):
            if file.endswith("ani.sta"):
                with open(self.directory+file) as f:
                    for line in f:
                        self.animal_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        self.after(100,self.update)


def delete_files():
    for file in os.listdir("./"):
        if file.endswith(".sta") or file.endswith(".ls") or file.endswith(".que"):
            os.remove("./" + file)
            print(str(file) + " Deleted" )

if __name__ == '__main__':
    gui = GUI_overlay()
    t1 = threading.Thread(target=gui.update_lasers, args=[])
    t1.start()
    gui.after(0,gui.update)
    gui.mainloop()
