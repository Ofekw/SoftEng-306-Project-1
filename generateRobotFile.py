#!/usr/bin/python
import subprocess
import os
import stat
import atexit

initial_x = -20
#Types of robots that the script reads the config file for
robot_type = ["Picker", "Carrier"]
#Loads the fields in the config file
config = {}
with open("config.properties", "r") as f:
    for line in f:
        property = line.split('=')
        config[property[0]] = property[1]

#Creates or overwrites the myworld.world file
myworld = open('world/myworld.world','w')
#Reads and add the world template to the world file to be written
world_template = open('world/templates/myworld.template').read()
myworld.write(world_template)
#Directory of where the temporary robot .py files will be written
directory = "./se306Project1/src/"

#List of the temporary robot files created
file_name = []
total_robots = 0;
#Each robot type will start at a different y-position
for type in robot_type:
    if type == "Picker":
        initial_y = "-28"
    else:
        initial_y = "-32"
    #Loads the corresponding robot template
    string = open('world/templates/' + type + '.template').read()
    number = config.get(type + '.number')
    robot = ""
    for i in range(0, int(number)):
        #Have to change the picker name when there's a model for the other types
        #Appends to the myworld file the robot model of each robot
        robot = robot + type.lower() + "( pose [ " + str(initial_x+(i*10))  +  " " + initial_y + " 0.000 90 ] name \"r" + str(total_robots) + "\")" + "\n"
        #The constructor of that robot type with the robot_id and the x y positions
        constructor = "    robot = Robot" + type + "(" + str(total_robots) + ", " + str(initial_x+(i*10)) + ", -28, math.pi/2)"
        name = "r" + str(total_robots) + ".py" #Name of the robot files
        file_name.append(name)
        temp = open(os.path.join(directory, name),'w')
        #Replaces "@@@" string in the template with the constructor
        temp.write(string.replace("@@@", constructor))
        temp.close()
        #Gives the temporary robot file run permission
        os.chmod(directory+name,stat.S_IRWXU)
        total_robots += 1
    robot = robot + "\n"
    #Writes the robot models to the world file
    myworld.write(robot)

myworld.close()

for name in file_name:
    #Runs all the temporary robot files created
    command = "bash -c 'sleep 3 && rosrun se306Project1 " + name + "'"
    subprocess.Popen(command, shell=True)

def delete_files():
    #This function is called when the process is killed. The function will delete all the temporary robot files and the myworld.world file
    os.remove('world/myworld.world')
    for name in file_name:
        os.remove(directory+name)

atexit.register(delete_files)

while True:
 pass