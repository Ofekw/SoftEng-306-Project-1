#!/usr/bin/python
import subprocess
import time
import getopt
import sys

debugger = False
try:
    opts, args = getopt.getopt(sys.argv[1:],"d")
except getopt.GetoptError:
    print 'test.py -d for debugger mode '
    sys.exit(2)
for opt, arg in opts:
    if opt == "-d":
        debugger = True
if debugger == True:
    subprocess.Popen("bash -c 'sleep 2 && python generateRobotFile.py -d'", shell=True)
    time.sleep(5)
    subprocess.Popen("bash -c 'sleep 2 && python generateWorldFile.py'", shell=True)
else:
    subprocess.Popen("bash -c 'sleep 2 && python generateRobotFile.py'", shell=True)
    time.sleep(5)
    subprocess.Popen("bash -c 'sleep 2 && python generateWorldFile.py'", shell=True)
    #subprocess.check_output("rosmake se306Project1", shell=True)
    subprocess.Popen("bash -c 'roscore'", shell=True)
    subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)
    time.sleep(3)
    #execute GUI script
    subprocess.call("./run_gui.sh")
