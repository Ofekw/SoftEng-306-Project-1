#!/usr/bin/python
import subprocess
import os
import time

#run the config related scripts
subprocess.check_output("gnome-terminal -x bash -c 'python generateWorldFile.py'", shell=True)

print subprocess.check_output("sleep 1 && rosmake se306Project1", shell=True)
subprocess.check_output("gnome-terminal -x bash -c 'roscore'", shell=True)
subprocess.check_output("gnome-terminal -x bash -c 'sleep 3 && rosrun stage_ros stageros world/myworld.world && sleep 5'", shell=True)
#we must give ros world enough time to load as there are lots of entities to load in
#run robots listed in robotList.txt file
with open("robotList.txt", "r") as f:
    for line in f:
        if line.startswith("#"):
            continue
        else:
	    command = "gnome-terminal -x bash -c 'sleep 3 && rosrun se306Project1 " +line+"'"
            subprocess.check_output(command, shell=True)


time.sleep(3)
#execute GUI script
subprocess.call("./run_gui.sh")



