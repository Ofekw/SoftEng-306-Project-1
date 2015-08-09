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
        geom = "500x800+603+196"
        self.geometry(geom)
        self.label_list = list()

        _bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        _fgcolor = '#0066FF'  # X11 color: 'black'
        _compcolor = '#d9d9d9' # X11 color: 'gray85'
        _ana1color = '#d9d9d9' # X11 color: 'gray85'
        _ana2color = '#d9d9d9' # X11 color: 'gray85'
        self.style = ttk.Style()
        if sys.platform == "win32":
            self.style.theme_use('winnative')
        self.style.configure('.',background=_bgcolor)
        self.style.configure('.',foreground=_fgcolor)
        self.style.configure('.',font="TkDefaultFont")
        self.style.map('.',background=
            [('selected', _compcolor), ('active',_ana2color)])
        self.configure(highlightcolor="black")


        self.TLabelframe1 = ttk.Labelframe(self)
        self.TLabelframe1.place(relx=0.02, rely=0.03, relheight=0.17
                , relwidth=0.96)
        self.TLabelframe1.configure(relief=RAISED)
        self.TLabelframe1.configure(text='''Robot Report''')
        self.TLabelframe1.configure(relief=RAISED)
        self.TLabelframe1.configure(width=390)

        self.Label1 = Label(self.TLabelframe1)
        self.Label1.place(relx=0.025, rely=0.15, height=19, width=74)
        self.Label1.configure(text='''Name (ID):''')

        self.Label2 = Label(self.TLabelframe1)
        self.Label2.place(relx=0.54, rely=0.07, height=19, width=66)
        self.Label2.configure(text='''X Position:''')

        self.Label3 = Label(self.TLabelframe1)
        self.Label3.place(relx=0.54, rely=0.22, height=19, width=65)
        self.Label3.configure(text='''Y Position:''')

        self.Label4 = Label(self.TLabelframe1)
        self.Label4.place(relx=0.54, rely=0.37, height=19, width=85)
        self.Label4.configure(text='''Theta (Rads):''')

        self.Label5 = Label(self.TLabelframe1)
        self.Label5.place(relx=0.03, rely=0.3, height=19, width=37)
        self.Label5.configure(text='''Type:''')

        self.Label6 = Label(self.TLabelframe1)
        self.Label6.place(relx=0.03, rely=0.52, height=19, width=95)
        self.Label6.configure(text='''Current action:''')

        self.Label7 = Label(self.TLabelframe1)
        self.Label7.place(relx=0.03, rely=0.74, height=19, width=109)
        self.Label7.configure(text='''Current capacity:''')



        self.TLabel1 = ttk.Label(self.TLabelframe1)
        self.TLabel1.place(relx=0.26, rely=0.15, height=17, width=50)
        self.TLabel1.configure(background=_bgcolor)
        self.TLabel1.configure(foreground="#0066FF")
        self.TLabel1.configure(relief=FLAT)

        self.TLabel2 = ttk.Label(self.TLabelframe1)
        self.TLabel2.place(relx=0.26, rely=0.3, height=17, width=50)
        self.TLabel2.configure(background=_bgcolor)
        self.TLabel2.configure(foreground="#0066FF")
        self.TLabel2.configure(relief=FLAT)

        self.TLabel3 = ttk.Label(self.TLabelframe1)
        self.TLabel3.place(relx=0.31, rely=0.52, height=17, width=50)
        self.TLabel3.configure(background=_bgcolor)
        self.TLabel3.configure(foreground="#0066FF")
        self.TLabel3.configure(relief=FLAT)

        self.TLabel4 = ttk.Label(self.TLabelframe1)
        self.TLabel4.place(relx=0.79, rely=0.07, height=17, width=50)
        self.TLabel4.configure(background=_bgcolor)
        self.TLabel4.configure(foreground="#006600")
        self.TLabel4.configure(relief=FLAT)

        self.TLabel5 = ttk.Label(self.TLabelframe1)
        self.TLabel5.place(relx=0.79, rely=0.22, height=17, width=50)
        self.TLabel5.configure(background=_bgcolor)
        self.TLabel5.configure(foreground="#006600")
        self.TLabel5.configure(relief=FLAT)

        self.TLabel6 = ttk.Label(self.TLabelframe1)
        self.TLabel6.place(relx=0.79, rely=0.37, height=17, width=50)
        self.TLabel6.configure(background=_bgcolor)
        self.TLabel6.configure(foreground="#006600")
        self.TLabel6.configure(relief=FLAT)

        self.TLabelframe2 = ttk.Labelframe(self)
        self.TLabelframe2.place(relx=0.02, rely=0.22, relheight=0.17
                , relwidth=0.96)
        self.TLabelframe2.configure(relief=RAISED)
        self.TLabelframe2.configure(text='''Robot Report''')
        self.TLabelframe2.configure(relief=RAISED)
        self.TLabelframe2.configure(width=390)

        self.Label8 = Label(self.TLabelframe2)
        self.Label8.place(relx=0.025, rely=0.15, height=19, width=74)
        self.Label8.configure(activebackground="#f9f9f9")
        self.Label8.configure(text='''Name (ID):''')

        self.Label9 = Label(self.TLabelframe2)
        self.Label9.place(relx=0.54, rely=0.07, height=19, width=66)
        self.Label9.configure(activebackground="#f9f9f9")
        self.Label9.configure(text='''X Position:''')

        self.Label10 = Label(self.TLabelframe2)
        self.Label10.place(relx=0.54, rely=0.22, height=19, width=65)
        self.Label10.configure(activebackground="#f9f9f9")
        self.Label10.configure(text='''Y Position:''')

        self.Label11 = Label(self.TLabelframe2)
        self.Label11.place(relx=0.54, rely=0.37, height=19, width=85)
        self.Label11.configure(activebackground="#f9f9f9")
        self.Label11.configure(text='''Theta (Rads):''')

        self.Label12 = Label(self.TLabelframe2)
        self.Label12.place(relx=0.03, rely=0.3, height=19, width=37)
        self.Label12.configure(activebackground="#f9f9f9")
        self.Label12.configure(text='''Type:''')

        self.Label13 = Label(self.TLabelframe2)
        self.Label13.place(relx=0.03, rely=0.52, height=19, width=95)
        self.Label13.configure(activebackground="#f9f9f9")
        self.Label13.configure(text='''Current action:''')

        self.Label14 = Label(self.TLabelframe2)
        self.Label14.place(relx=0.03, rely=0.74, height=19, width=109)
        self.Label14.configure(activebackground="#f9f9f9")
        self.Label14.configure(text='''Current capacity:''')


        self.TLabel7 = ttk.Label(self.TLabelframe2)
        self.TLabel7.place(relx=0.26, rely=0.15, height=17, width=50)
        self.TLabel7.configure(background=_bgcolor)
        self.TLabel7.configure(foreground="#0066FF")
        self.TLabel7.configure(relief=FLAT)

        self.TLabel8 = ttk.Label(self.TLabelframe2)
        self.TLabel8.place(relx=0.26, rely=0.3, height=17, width=50)
        self.TLabel8.configure(background=_bgcolor)
        self.TLabel8.configure(foreground="#0066FF")
        self.TLabel8.configure(relief=FLAT)

        self.TLabel9 = ttk.Label(self.TLabelframe2)
        self.TLabel9.place(relx=0.31, rely=0.52, height=17, width=50)
        self.TLabel9.configure(background=_bgcolor)
        self.TLabel9.configure(foreground="#0066FF")
        self.TLabel9.configure(relief=FLAT)

        self.TLabel10 = ttk.Label(self.TLabelframe2)
        self.TLabel10.place(relx=0.79, rely=0.07, height=17, width=50)
        self.TLabel10.configure(background=_bgcolor)
        self.TLabel10.configure(foreground="#006600")
        self.TLabel10.configure(relief=FLAT)

        self.TLabel11 = ttk.Label(self.TLabelframe2)
        self.TLabel11.place(relx=0.79, rely=0.22, height=17, width=50)
        self.TLabel11.configure(background=_bgcolor)
        self.TLabel11.configure(foreground="#006600")
        self.TLabel11.configure(relief=FLAT)

        self.TLabel12 = ttk.Label(self.TLabelframe2)
        self.TLabel12.place(relx=0.79, rely=0.37, height=17, width=50)
        self.TLabel12.configure(background=_bgcolor)
        self.TLabel12.configure(foreground="#006600")
        self.TLabel12.configure(relief=FLAT)

        self.TLabelframe3 = ttk.Labelframe(self)
        self.TLabelframe3.place(relx=0.02, rely=0.41, relheight=0.17
                , relwidth=0.96)
        self.TLabelframe3.configure(relief=RAISED)
        self.TLabelframe3.configure(text="Robot Report")
        self.TLabelframe3.configure(relief=RAISED)
        self.TLabelframe3.configure(width=390)

        self.Label15 = Label(self.TLabelframe3)
        self.Label15.place(relx=0.025, rely=0.15, height=19, width=74)
        self.Label15.configure(activebackground="#f9f9f9")
        self.Label15.configure(text='''Name (ID):''')

        self.Label16 = Label(self.TLabelframe3)
        self.Label16.place(relx=0.54, rely=0.07, height=19, width=66)
        self.Label16.configure(activebackground="#f9f9f9")
        self.Label16.configure(text='''X Position:''')

        self.Label17 = Label(self.TLabelframe3)
        self.Label17.place(relx=0.54, rely=0.22, height=19, width=65)
        self.Label17.configure(activebackground="#f9f9f9")
        self.Label17.configure(text='''Y Position:''')

        self.Label18 = Label(self.TLabelframe3)
        self.Label18.place(relx=0.54, rely=0.37, height=19, width=85)
        self.Label18.configure(activebackground="#f9f9f9")
        self.Label18.configure(text='''Theta (Rads):''')

        self.Label19 = Label(self.TLabelframe3)
        self.Label19.place(relx=0.03, rely=0.3, height=19, width=37)
        self.Label19.configure(activebackground="#f9f9f9")
        self.Label19.configure(text='''Type:''')

        self.Label20 = Label(self.TLabelframe3)
        self.Label20.place(relx=0.03, rely=0.52, height=19, width=95)
        self.Label20.configure(activebackground="#f9f9f9")
        self.Label20.configure(text='''Current action:''')

        self.Label21 = Label(self.TLabelframe3)
        self.Label21.place(relx=0.03, rely=0.74, height=19, width=109)
        self.Label21.configure(activebackground="#f9f9f9")
        self.Label21.configure(text='''Current capacity:''')



        self.TLabel13 = ttk.Label(self.TLabelframe3)
        self.TLabel13.place(relx=0.26, rely=0.15, height=17, width=50)
        self.TLabel13.configure(background=_bgcolor)
        self.TLabel13.configure(foreground="#0066FF")
        self.TLabel13.configure(relief=FLAT)

        self.TLabel14 = ttk.Label(self.TLabelframe3)
        self.TLabel14.place(relx=0.26, rely=0.3, height=17, width=50)
        self.TLabel14.configure(background=_bgcolor)
        self.TLabel14.configure(foreground="#0066FF")
        self.TLabel14.configure(relief=FLAT)

        self.TLabel15 = ttk.Label(self.TLabelframe3)
        self.TLabel15.place(relx=0.31, rely=0.52, height=17, width=50)
        self.TLabel15.configure(background=_bgcolor)
        self.TLabel15.configure(foreground="#0066FF")
        self.TLabel15.configure(relief=FLAT)

        self.TLabel16 = ttk.Label(self.TLabelframe3)
        self.TLabel16.place(relx=0.79, rely=0.07, height=17, width=50)
        self.TLabel16.configure(background=_bgcolor)
        self.TLabel16.configure(foreground="#006600")
        self.TLabel16.configure(relief=FLAT)

        self.TLabel17 = ttk.Label(self.TLabelframe3)
        self.TLabel17.place(relx=0.79, rely=0.22, height=17, width=50)
        self.TLabel17.configure(background=_bgcolor)
        self.TLabel17.configure(foreground="#006600")
        self.TLabel17.configure(relief=FLAT)

        self.TLabel18 = ttk.Label(self.TLabelframe3)
        self.TLabel18.place(relx=0.79, rely=0.37, height=17, width=50)
        self.TLabel18.configure(background=_bgcolor)
        self.TLabel18.configure(foreground="#006600")
        self.TLabel18.configure(relief=FLAT)

        self.TLabelframe4 = ttk.Labelframe(self)
        self.TLabelframe4.place(relx=0.02, rely=0.6, relheight=0.17
                , relwidth=0.96)
        self.TLabelframe4.configure(relief=RAISED)
        self.TLabelframe4.configure(text='''Robot Report''')
        self.TLabelframe4.configure(relief=RAISED)
        self.TLabelframe4.configure(width=390)

        self.Label22 = Label(self.TLabelframe4)
        self.Label22.place(relx=0.025, rely=0.15, height=19, width=74)
        self.Label22.configure(activebackground="#f9f9f9")
        self.Label22.configure(text='''Name (ID):''')

        self.Label23 = Label(self.TLabelframe4)
        self.Label23.place(relx=0.54, rely=0.07, height=19, width=66)
        self.Label23.configure(activebackground="#f9f9f9")
        self.Label23.configure(text='''X Position:''')

        self.Label24 = Label(self.TLabelframe4)
        self.Label24.place(relx=0.54, rely=0.22, height=19, width=65)
        self.Label24.configure(activebackground="#f9f9f9")
        self.Label24.configure(text='''Y Position:''')

        self.Label25 = Label(self.TLabelframe4)
        self.Label25.place(relx=0.54, rely=0.37, height=19, width=85)
        self.Label25.configure(activebackground="#f9f9f9")
        self.Label25.configure(text='''Theta (Rads):''')

        self.Label26 = Label(self.TLabelframe4)
        self.Label26.place(relx=0.03, rely=0.3, height=19, width=37)
        self.Label26.configure(activebackground="#f9f9f9")
        self.Label26.configure(text='''Type:''')

        self.Label27 = Label(self.TLabelframe4)
        self.Label27.place(relx=0.03, rely=0.52, height=19, width=95)
        self.Label27.configure(activebackground="#f9f9f9")
        self.Label27.configure(text='''Current action:''')

        self.Label28 = Label(self.TLabelframe4)
        self.Label28.place(relx=0.03, rely=0.74, height=19, width=109)
        self.Label28.configure(activebackground="#f9f9f9")
        self.Label28.configure(text='''Current capacity:''')



        self.TLabel19 = ttk.Label(self.TLabelframe4)
        self.TLabel19.place(relx=0.26, rely=0.15, height=17, width=50)
        self.TLabel19.configure(background=_bgcolor)
        self.TLabel19.configure(foreground="#0066FF")
        self.TLabel19.configure(relief=FLAT)

        self.TLabel20 = ttk.Label(self.TLabelframe4)
        self.TLabel20.place(relx=0.26, rely=0.3, height=17, width=50)
        self.TLabel20.configure(background=_bgcolor)
        self.TLabel20.configure(foreground="#0066FF")
        self.TLabel20.configure(relief=FLAT)

        self.TLabel21 = ttk.Label(self.TLabelframe4)
        self.TLabel21.place(relx=0.31, rely=0.52, height=17, width=50)
        self.TLabel21.configure(background=_bgcolor)
        self.TLabel21.configure(foreground="#0066FF")
        self.TLabel21.configure(relief=FLAT)

        self.TLabel22 = ttk.Label(self.TLabelframe4)
        self.TLabel22.place(relx=0.79, rely=0.07, height=17, width=50)
        self.TLabel22.configure(background=_bgcolor)
        self.TLabel22.configure(foreground="#006600")
        self.TLabel22.configure(relief=FLAT)

        self.TLabel23 = ttk.Label(self.TLabelframe4)
        self.TLabel23.place(relx=0.79, rely=0.22, height=17, width=50)
        self.TLabel23.configure(background=_bgcolor)
        self.TLabel23.configure(foreground="#006600")
        self.TLabel23.configure(relief=FLAT)

        self.TLabel24 = ttk.Label(self.TLabelframe4)
        self.TLabel24.place(relx=0.79, rely=0.37, height=17, width=50)
        self.TLabel24.configure(background=_bgcolor)
        self.TLabel24.configure(foreground="#006600")
        self.TLabel24.configure(relief=FLAT)

        self.TLabelframe5 = ttk.Labelframe(self)
        self.TLabelframe5.place(relx=0.02, rely=0.8, relheight=0.17
                , relwidth=0.96)
        self.TLabelframe5.configure(relief=RAISED)
        self.TLabelframe5.configure(text='''Robot Report''')
        self.TLabelframe5.configure(relief=RAISED)
        self.TLabelframe5.configure(width=390)

        self.Label29 = Label(self.TLabelframe5)
        self.Label29.place(relx=0.025, rely=0.15, height=19, width=74)
        self.Label29.configure(activebackground="#f9f9f9")
        self.Label29.configure(text='''Name (ID):''')

        self.Label30 = Label(self.TLabelframe5)
        self.Label30.place(relx=0.54, rely=0.07, height=19, width=66)
        self.Label30.configure(activebackground="#f9f9f9")
        self.Label30.configure(text='''X Position:''')

        self.Label31 = Label(self.TLabelframe5)
        self.Label31.place(relx=0.54, rely=0.22, height=19, width=65)
        self.Label31.configure(activebackground="#f9f9f9")
        self.Label31.configure(text='''Y Position:''')

        self.Label32 = Label(self.TLabelframe5)
        self.Label32.place(relx=0.54, rely=0.37, height=19, width=85)
        self.Label32.configure(activebackground="#f9f9f9")
        self.Label32.configure(text='''Theta (Rads):''')

        self.Label33 = Label(self.TLabelframe5)
        self.Label33.place(relx=0.03, rely=0.3, height=19, width=50)
        self.Label33.configure(activebackground="#f9f9f9")
        self.Label33.configure(text='''Type:''')

        self.Label34 = Label(self.TLabelframe5)
        self.Label34.place(relx=0.03, rely=0.52, height=19, width=95)
        self.Label34.configure(activebackground="#f9f9f9")
        self.Label34.configure(text='''Current action:''')

        self.Label35 = Label(self.TLabelframe5)
        self.Label35.place(relx=0.03, rely=0.74, height=19, width=109)
        self.Label35.configure(activebackground="#f9f9f9")
        self.Label35.configure(text='''Current capacity:''')


        self.TLabel25 = ttk.Label(self.TLabelframe5)
        self.TLabel25.place(relx=0.26, rely=0.15, height=17, width=50)
        self.TLabel25.configure(background=_bgcolor)
        self.TLabel25.configure(foreground="#0066FF")
        self.TLabel25.configure(relief=FLAT)

        self.TLabel26 = ttk.Label(self.TLabelframe5)
        self.TLabel26.place(relx=0.26, rely=0.3, height=17, width=50)
        self.TLabel26.configure(background=_bgcolor)
        self.TLabel26.configure(foreground="#0066FF")
        self.TLabel26.configure(relief=FLAT)

        self.TLabel27 = ttk.Label(self.TLabelframe5)
        self.TLabel27.place(relx=0.31, rely=0.52, height=17, width=50)
        self.TLabel27.configure(background=_bgcolor)
        self.TLabel27.configure(foreground="#0066FF")
        self.TLabel27.configure(relief=FLAT)

        self.TLabel28 = ttk.Label(self.TLabelframe5)
        self.TLabel28.place(relx=0.79, rely=0.07, height=17, width=50)
        self.TLabel28.configure(background=_bgcolor)
        self.TLabel28.configure(foreground="#006600")
        self.TLabel28.configure(relief=FLAT)

        self.TLabel29 = ttk.Label(self.TLabelframe5)
        self.TLabel29.place(relx=0.79, rely=0.22, height=17, width=50)
        self.TLabel29.configure(background=_bgcolor)
        self.TLabel29.configure(foreground="#006600")
        self.TLabel29.configure(relief=FLAT)

        self.TLabel30 = ttk.Label(self.TLabelframe5)
        self.TLabel30.place(relx=0.79, rely=0.37, height=17, width=50)
        self.TLabel30.configure(background=_bgcolor)
        self.TLabel30.configure(foreground="#006600")
        self.TLabel30.configure(relief=FLAT)

        self.Label36 = Label(self.TLabelframe1)
        self.Label36.place(relx=0.33, rely=0.74, height=19, width=34)
        self.Label36.configure(cursor="fleur")
        self.Label36.configure(foreground="#FF0000")


        self.Label37 = Label(self.TLabelframe2)
        self.Label37.place(relx=0.31, rely=0.74, height=19, width=56)
        self.Label37.configure(width=56)
        self.Label37.configure(foreground="#FF0000")

        self.Label38 = Label(self.TLabelframe3)
        self.Label38.place(relx=0.33, rely=0.74, height=19, width=34)
        self.Label38.configure(foreground="#FF0000")

        self.Label39 = Label(self.TLabelframe4)
        self.Label39.place(relx=0.33, rely=0.74, height=19, width=34)
        self.Label39.configure(foreground="#FF0000")


        self.Label40 = Label(self.TLabelframe5)
        self.Label40.place(relx=0.33, rely=0.74, height=19, width=34)
        self.Label40.configure(foreground="#FF0000")

        self.label_list.append(self.TLabel1)
        self.label_list.append(self.TLabel2)
        self.label_list.append(self.TLabel3)
        self.label_list.append(self.TLabel4)
        self.label_list.append(self.TLabel5)
        self.label_list.append(self.TLabel6)
        self.label_list.append(self.Label36)
        self.label_list.append(self.TLabel7)
        self.label_list.append(self.TLabel8)
        self.label_list.append(self.TLabel9)
        self.label_list.append(self.TLabel10)
        self.label_list.append(self.TLabel11)
        self.label_list.append(self.TLabel12)
        self.label_list.append(self.Label37)
        self.label_list.append(self.TLabel13)
        self.label_list.append(self.TLabel14)
        self.label_list.append(self.TLabel15)
        self.label_list.append(self.TLabel16)
        self.label_list.append(self.TLabel17)
        self.label_list.append(self.TLabel18)
        self.label_list.append(self.Label38)
        self.label_list.append(self.TLabel19)
        self.label_list.append(self.TLabel20)
        self.label_list.append(self.TLabel21)
        self.label_list.append(self.TLabel22)
        self.label_list.append(self.TLabel23)
        self.label_list.append(self.TLabel24)
        self.label_list.append(self.Label39)
        self.label_list.append(self.TLabel25)
        self.label_list.append(self.TLabel26)
        self.label_list.append(self.TLabel27)
        self.label_list.append(self.TLabel28)
        self.label_list.append(self.TLabel29)
        self.label_list.append(self.TLabel30)
        self.label_list.append(self.Label40)


    def update(self):
        i = 0
        for file in os.listdir(self.directory):
            if file.endswith(".sta"):
                with open(file) as f:
                    for line in f:
                        self.label_list[i].configure(text=str(line))
                        i+=1
        i=0
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
