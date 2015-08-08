#!/usr/bin/python
import subprocess
import os
import stat
import atexit

subprocess.check_output("rosmake se306Project1", shell=True)
subprocess.Popen("bash -c 'roscore'", shell=True)
subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)
subprocess.Popen("bash -c 'sleep 2 && python generateRobotFile.py'", shell=True)


while True:
 pass