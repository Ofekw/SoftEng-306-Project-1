#!/usr/bin/python
import subprocess
import time
import getopt
import sys
import atexit
import generateEntity
import generateWorldFile

processes = []

def main(argv):
    testing = False
    debugging = False

    config = {}
    with open("config.properties", "r") as f:
        for line in f:
            property = line.split('=')
            config[property[0]] = property[1]
    try:
        opts, args = getopt.getopt(argv,"dt")
    except getopt.GetoptError:
        print 'test.py -d for debugger mode '
        print 'test.py -t for debugger mode '
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-t":
            testing = True
        elif opt == "-d":
            debugging = True
    if testing == True:
        list = generateEntity.main(['-t'], config)
        generateWorldFile.main(config)
        return list
    else:
        if debugging == True:
            list = generateEntity.main(["-d"])
        else:
            list = generateEntity.main([""], config)
        atexit.register(generateEntity.exit_process, list)
        generateWorldFile.main(config)
        processes.append(subprocess.Popen(["roscore"], shell=False))
        processes.append(subprocess.Popen(["rosrun", "stage_ros", "stageros", "world/myworld.world"], shell=False))
        time.sleep(5)
        atexit.register(kill_process)
        #execute GUI script
        processes.append(subprocess.Popen(['python', 'GUI_overlay.py'], cwd=r'./se306Project1/src'))
        atexit.register(kill_GUI)
        while True:
            pass

def kill_GUI():
    subprocess.Popen(['python', '-c', 'import GUI_overlay; GUI_overlay.delete_files()'], cwd=r'./se306Project1/src')

def kill_process():
    for i in range(0, processes.__len__()):
        processes[i].terminate()

if __name__ == "__main__":
    main(sys.argv[1:])
