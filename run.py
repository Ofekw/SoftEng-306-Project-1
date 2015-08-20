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
    webservice = False


    config = {}
    with open("config.properties", "r") as f:
        for line in f:
            property = line.split('=')
            config[property[0]] = property[1]
    try:
        opts, args = getopt.getopt(argv,"dtw")
    except getopt.GetoptError:
        print 'run.py -d for debugger mode '
        print 'run.py -t for testing mode '
        print 'run.py -w for webservice mode '
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-t":
            testing = True
        elif opt == "-d":
            debugging = True
        elif opt == "-w":
            webservice = True
	
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
        if webservice == True:
            web = subprocess.call("./run_webservice.sh")
        while True:
            pass

def kill_GUI():
    subprocess.Popen(['python', '-c', 'import GUI_overlay; GUI_overlay.delete_files()'], cwd=r'./se306Project1/src')

def kill_process():
    for i in range(0, processes.__len__()):
        processes[i].terminate()

if __name__ == "__main__":
    main(sys.argv[1:])
