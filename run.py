#!/usr/bin/python
import subprocess
import os
import stat
import atexit
import time

subprocess.Popen("bash -c 'sleep 2 && python generateRobotFile.py'", shell=True)
subprocess.Popen("bash -c 'sleep 2 && python generateWorldFile.py'", shell=True)
subprocess.check_output("rosmake se306Project1", shell=True)
subprocess.Popen("bash -c 'roscore'", shell=True)
subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)

time.sleep(3)
#execute GUI script
subprocess.call("./run_gui.sh")

