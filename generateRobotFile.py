#!/usr/bin/python
import subprocess
import os
import stat
import sys
import getopt

#Directory of where the temporary robot .py files will be written
directory = "./se306Project1/src/"
debug_text1 = "    debug = Debugger(robot)"
debug_text2 = "    debug.start()"

def main(argv):
    testing = False
    debugging = False
    try:
        opts, args = getopt.getopt(argv,"td")
    except getopt.GetoptError:
        print 'test.py -d for debugger mode '
        print 'test.py -t for debugger mode '
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-t':
            testing = True
        elif opt == '-d':
            debugging = True

    #List of the temporary robot files created
    file_name = []
    initial_x = -20
    #Types of robots that the script reads the config file for
    robot_type = ["Picker", "Carrier", "Visitor", "Animal"]
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

    total_robots = 0;
    #Each robot type will start at a different y-position
    for type in robot_type:
        if type == "Picker":
            initial_y = "-28"
        elif type == "Carrier":
            initial_y = "-32"
    	elif type == "Visitor":
        	initial_y = "-34"
        else:
            initial_y = "-38"
        #Loads the corresponding robot template
        string = open('world/templates/' + type + '.template').read()
        number = config.get(type + '.number')
        robot = ""
        for i in range(0, int(number)):
            #Appends to the myworld file the robot model of each robot
            robot = robot + type.lower() + "( pose [ " + str(initial_x+(i*10))  +  " " + initial_y + " 0.000 90 ] name \"r" + str(total_robots) + "\")" + "\n"
            #The constructor of that robot type with the robot_id and the x y positions
            constructor_name = "Robot" + type
            if type == "Visitor" or type == "Animal":
                constructor_name = constructor_name.replace("Robot", "")
            constructor = "    robot = " + constructor_name + "(" + str(total_robots) + ", " + str(initial_x+(i*10)) + ", -28, math.pi/2)"
            if debugging == True:
                constructor = constructor + "\n" + debug_text1 + "\n" + debug_text2
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

    if testing == False:
        for name in file_name:
            #Runs all the temporary robot files created
            command = "bash -c 'sleep 3 && rosrun se306Project1 " + name + "'"
            subprocess.Popen(command, shell=True)
    return file_name

def delete_files(file_name):
    #This function is called when the process is killed. The function will delete all the temporary robot files and the myworld.world file
    os.remove('./world/myworld.world')
    for name in file_name:
        os.remove(directory+name)

if __name__ == "__main__":
    main(sys.argv[1:])


