import subprocess
import sys
import time
from os import listdir
from os.path import isfile, join
import time

def main(argv):
    global log
    processes = []
    mypath = 'se306Project1/test'
    test_files = []
    for f in listdir(mypath):
        if isfile(join(mypath,f)):
            if f.startswith('Test_'):
                test_files.append(f)

    #logging
    log = open('test.log' ,'w+')

    #build and generation test
    test_build()

    #start ros services
    start_services()

    #generate entities
    setup(processes)

    #run unit tests
    run_tests(processes, test_files)

def test_build():
    p = subprocess.Popen("bash -c 'python TestGenerateFiles.py'", shell=True, stdout=log, stderr=log)
    p.wait()
    if p.returncode == 0:
        print("SUCCESS build tests all passed")

def start_services():
    subprocess.check_output("rosmake se306Project1", shell=True)
    subprocess.Popen("bash -c 'python generateWorldFile.py'", shell=True)
    subprocess.Popen("bash -c 'roscore'", shell=True)

def setup(processes):
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros -g world/myworld.backup.world'", shell=True))

def cleanup(processes):
    for i in processes:
        i.kill()
    processes=[]

def run_tests(processes, test_files):
    time.sleep(10)
    spinner = spinning_cursor()
    for file in test_files:
        print("TESTING: " + file +'\n')
        p = subprocess.Popen("bash -c 'sleep 1 && rosrun se306Project1 "+file+"'", shell=True,stdout=log, stderr=log)
        for i in range(0,50):
            sys.stdout.write(spinner.next())
            sys.stdout.flush()
            time.sleep(0.1)
            sys.stdout.write('\b')
        p.wait()
        if p.returncode == 0:
            cleanup(processes)
            setup(processes)
            time.sleep(5)

    cleanup(processes)

def spinning_cursor():
    while True:
        for cursor in '|/-\\':
            yield cursor


if __name__ == "__main__":
    main(sys.argv[1:])
