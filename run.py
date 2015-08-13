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

    def cleanup():
      os.system("ps aux | grep python | grep -v 'grep python' | awk '{print $2}' | xargs kill -9")

    atexit.register(cleanup)
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
        time.sleep(3)
        #execute GUI script
        gui = subprocess.call("./run_gui.sh")
        while True:
            pass


if __name__ == "__main__":
    main(sys.argv[1:])
