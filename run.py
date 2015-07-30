#!/usr/bin/python
import subprocess

print subprocess.check_output("rosmake se306Project1", shell=True)
subprocess.check_output("gnome-terminal -x bash -c 'roscore'", shell=True)
subprocess.check_output("gnome-terminal -x bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)
subprocess.check_output("gnome-terminal -x bash -c 'sleep 3 && rosrun se306Project1 R0.py'", shell=True)

#run robots listed in robotList.txt file
with open("robotList.txt", "r") as f:
    for line in f:
        subprocess.check_output("gnome-terminal -x bash -c 'sleep 3 && rosrun se306Project1 '"+line, shell=True)
