#!/usr/bin/env python

import sys
import threading
import Tkinter
import multiprocessing
import os
import atexit


from Tkinter import *
import ttk


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

        self.title('Mission Control: Fl0PPY D15K')
        geom = "500x850+603+196"
        self.geometry(geom)
        self.label_list = list()

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
        human_tab = Frame(nb)
        animal_tab = Frame(nb)

        # create the tabs
        nb.add(robot_tab, text='Robot')
        nb.add(human_tab, text='Humans')
        nb.add(animal_tab, text='Animals')

        #BEGIN ROBOTS

        self.robot_label_list = self.setup_robots(nb,robot_tab)

        # BEGIN HUMANS ________________________________________________________
        self.human_label_list = self.setup_humans(nb,human_tab)

        # BEGIN ANIMALS _______________________________________________________-
        self.animal_label_list = self.setup_animals(nb,animal_tab)


    def setup_robots(self,nb,robot_tab):

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



        human_label_list = [human_tab.TLabel1,human_tab.TLabel2,human_tab.TLabel3,human_tab.TLabel4,human_tab.TLabel5,human_tab.TLabel6,human_tab.TLabel8,human_tab.TLabel9,human_tab.TLabel10,human_tab.TLabel11]

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
        animal_tab.TLabel9.place(relx=0.31, rely=0.6, height=17, width=50)
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


        animal_label_list = [animal_tab.TLabel1,animal_tab.TLabel2,animal_tab.TLabel3,animal_tab.TLabel4,animal_tab.TLabel5,animal_tab.TLabel6,animal_tab.TLabel8,animal_tab.TLabel9,animal_tab.TLabel10,animal_tab.TLabel11,animal_tab.TLabel12]

        return animal_label_list


    def update(self):
        i = 0
        for file in os.listdir(self.directory):
            if file.endswith("pic.sta")or file.endswith("car.sta"):
                with open(file) as f:
                    for line in f:
                        self.robot_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        for file in os.listdir(self.directory):
            if file.endswith("vis.sta"):
                with open(file) as f:
                    for line in f:
                        self.human_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        for file in os.listdir(self.directory):
            if file.endswith("ani.sta"):
                with open(file) as f:
                    for line in f:
                        self.animal_label_list[i].configure(text=str(line))
                        i+=1
        i=0
        self.after(200,self.update)

def delete_files():
    for file in os.listdir("./"):
        if file.endswith(".sta"):
            print(str(file) + " Deleted" )
            os.remove("./" + file)

if __name__ == '__main__':
    gui = GUI_overlay()
    gui.after(0,gui.update)
    gui.mainloop()
