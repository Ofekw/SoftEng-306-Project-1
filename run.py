#!/usr/bin/python
import subprocess
import time
import getopt
import sys
import atexit
import generateEntity
import os

def main(argv):
    testing = False
    debugging = False
    webservice = False


    def cleanup():
      os.system("ps aux | grep python | grep -v 'grep python' | awk '{print $2}' | xargs kill -9")

    atexit.register(cleanup)
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
        list = generateEntity.main(['-t'])
        process = subprocess.Popen("bash -c 'python generateWorldFile.py'", shell=True)
        process.wait()
        return list
    else:
        if debugging == True:
            list = generateEntity.main(["-d"])
        else:
            list = generateEntity.main([""])
        atexit.register(generateEntity.delete_files, list)
        subprocess.Popen("bash -c 'python generateWorldFile.py'", shell=True)
        subprocess.check_output("rosmake se306Project1", shell=True)
        subprocess.Popen("bash -c 'roscore'", shell=True)
        subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.world'", shell=True)
        time.sleep(5)
        #execute GUI script
        gui = subprocess.call("./run_gui.sh")
        if webservice == True:
	  web = subprocess.call("./run_webservice.sh")
        while True:
            pass


if __name__ == "__main__":
    main(sys.argv[1:])
