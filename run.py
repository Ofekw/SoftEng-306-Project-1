#!/usr/bin/python
import subprocess
import os
import stat
import atexit

print subprocess.check_output("rosmake se306Project1", shell=True)
subprocess.Popen("bash -c 'roscore'", shell=True)
subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)

number = 0
colors = []
#run robots listed in robotList.txt file
file = open("robotList.txt", "r")
line = file.readline()
while line not in ['\n', '\n\r']:
    args = line.split()
    line = file.readline()



string = ""
with open("header.txt", "r") as f:
    for line in f:
        string += line

directory = "./se306Project1/src/"
name = "r1.py"
temp = open(os.path.join(directory, name),'a')
temp.write(string)
temp.close()
os.chmod(directory+name,stat.S_IRWXU)
command = "bash -c 'sleep 3 && rosrun se306Project1 " + name + "'"
subprocess.Popen(command, shell=True)

def delete_files():
    os.remove(directory+name)

atexit.register(delete_files)

while True:
    pass