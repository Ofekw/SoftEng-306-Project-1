import subprocess
import sys
import os
import atexit
from os import listdir
from os.path import isfile, join
import time

def main(argv):
    processes = []
    mypath = 'se306Project1/test'
    test_files = []
    for f in listdir(mypath):
        if isfile(join(mypath,f)):
            if f.startswith('Test_'):
                test_files.append(f)

    start_services()
    setup(processes)
    run_tests(processes, test_files)



def start_services():
    subprocess.check_output("rosmake se306Project1", shell=True)
    subprocess.Popen("bash -c 'python generateWorldFile.py'", shell=True)
    subprocess.Popen("bash -c 'roscore'", shell=True)

def setup(processes):
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros -g world/myworld.backup.world'", shell=True))

    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Picker0_test.py'", shell=True))
    # processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Carrier0_test.py'", shell=True))
    # processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Animal0_test.py'", shell=True))
    # processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Visitor0_test.py'", shell=True))

def cleanup(processes):
    for i in processes:
        i.kill()
    processes=[]

def run_tests(processes, test_files):
    time.sleep(5)
    for file in test_files:
        p = subprocess.Popen("bash -c 'sleep 1 && rosrun se306Project1 "+file+"'", shell=True)
        p.wait()
        if p.returncode == 0:
            cleanup(processes)
            setup(processes)
            time.sleep(5)

    cleanup(processes)

if __name__ == "__main__":
    main(sys.argv[1:])
