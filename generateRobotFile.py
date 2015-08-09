#!/usr/bin/python
import subprocess
import os
import stat
import atexit

initial_x = -20
config = {}
with open("config.properties", "r") as f:
    for line in f:
        property = line.split('=')
        config[property[0]] = property[1]
number = config.get('robot.number')

robot = ""
for i in range(0, int(number)):
    robot = robot + "picker( pose [ " + str(initial_x+(i*10))  + " -28 0.000 90 ] name \"r" + str(i) + "\" color \"yellow\")" + "\n"
robot = robot + "\n"
myworld = open('world/myworld.world','w')
world_template = open('world/templates/myworld.template').read()
myworld.write(world_template)
myworld.write(robot)
myworld.close()

string = open('world/templates/robot.template').read()

file_name = []
directory = "./se306Project1/src/"
for i in range(0, int(number)):
    replace = "    robot = RobotPicker(" + str(initial_x+(i*10)) + ", -28, 0, math.pi/2)"
    name = "r" + str(i) + ".py"
    file_name.append(name)
    temp = open(os.path.join(directory, name),'w')
    temp.write(string.replace("@@@", replace))
    temp.close()
    os.chmod(directory+name,stat.S_IRWXU)

for name in file_name:
    command = "bash -c 'sleep 3 && rosrun se306Project1 " + name + "'"
    subprocess.Popen(command, shell=True)

def delete_files():
    os.remove('world/myworld.world')
    for name in file_name:
        os.remove(directory+name)

atexit.register(delete_files)

while True:
 pass