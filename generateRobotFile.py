#!/usr/bin/python
import subprocess
import os
import stat
import atexit

config = {}
with open("config.properties", "r") as f:
    for line in f:
        property = line.split('=')
        config[property[0]] = property[1]
number = config.get('robot.number')

string = ""
with open("header.txt", "r") as f:
    for line in f:
        string += line

file_name = []
directory = "./se306Project1/src/"
for i in range(0, int(number)):
    replace = "    robot = RobotPicker(" + str(i) + ", 0, 0, math.pi/2)"
    name = "r" + str(i) + ".py"
    file_name.append(name)
    temp = open(os.path.join(directory, name),'a')
    temp.write(string.replace("@@@", replace))
    temp.close()
    os.chmod(directory+name,stat.S_IRWXU)

for name in file_name:
    command = "bash -c 'sleep 3 && rosrun se306Project1 " + name + "'"
    subprocess.Popen(command, shell=True)

def delete_files():
    for name in file_name:
        os.remove(directory+name)

atexit.register(delete_files)

while True:
 pass