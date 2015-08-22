import subprocess
import sys
import time
from os import listdir
from os.path import isfile, join, devnull
import time
import generateWorldFile

config = {}
def main(argv):
    print "\n-----------------------------------------------------"
    print "                Floppy Disk Testing Unit"
    print "-----------------------------------------------------\n"
    with open("config.properties", "r") as f:
        for line in f:
            property = line.split('=')
            config[property[0]] = property[1]
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
    start_services(processes)

    #generate entities
    setup(processes)

    #run unit tests
    run_tests(processes, test_files)

def test_build():
    print "Build Script Testing:\n"
    p = subprocess.Popen(['python', 'TestGenerateFiles.py'], shell=False, stdout=log, stderr=log)
    p.wait()
    if p.returncode == 0:
        print("\tTesting Completed\n")

def start_services(processes):
    generateWorldFile.main(config)
    FNULL = open(devnull, 'w')
    processes.append(subprocess.Popen(['roscore'], shell=False, stdout=FNULL, stderr=FNULL))

def setup(processes):
    FNULL = open(devnull, 'w')
    processes.append(subprocess.Popen(['rosrun', 'stage_ros', 'stageros', '-g', 'world/myworld.backup.world'], shell=False, stdout=FNULL, stderr=subprocess.STDOUT))

def cleanup(processes):
    for i in processes:
        i.kill()
    processes=[]

def run_tests(processes, test_files):
    print "Entity and GUI Testing:\n"
    time.sleep(10)
    spinner = spinning_cursor()
    test_files.sort()
    for file in test_files:
        print("\tTESTING: " + file +'\n')
        s = subprocess.call(["chmod","+x",'se306Project1/test/'+file])
        p = subprocess.Popen(['rosrun', 'se306Project1', file], shell=False, stdout=log, stderr=log)
        for i in range(0,50):
            sys.stdout.write(spinner.next())
            sys.stdout.flush()
            time.sleep(0.1)
            sys.stdout.write('\b')
        p.wait()
        if p.returncode == 0:
            print "\t\t Testing Completed:\n"
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
