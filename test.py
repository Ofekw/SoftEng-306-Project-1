import subprocess
import sys
import os
import atexit

def main(argv):
    processes = []
    start_services()
    setup(processes)
    run_tests()
    cleanup(processes)



def start_services():
    subprocess.check_output("rosmake se306Project1", shell=True)
    subprocess.Popen("bash -c 'python generateWorldFile.py'", shell=True)
    subprocess.Popen("bash -c 'roscore'", shell=True)

def setup(processes):
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun stage_ros stageros world/myworld.backup.world'", shell=True))

    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Picker0_test.py'", shell=True))
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Carrier0_test.py'", shell=True))
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Animal0_test.py'", shell=True))
    processes.append(subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Visitor0_test.py'", shell=True))

def cleanup(processes):
    print("getting here")
    for i in processes:
        i.kill()
    processes=[]

def run_tests():
    p = subprocess.Popen("bash -c 'sleep 2 && rosrun se306Project1 Test_Robot_face_direction.py'", shell=True)
    p.communicate()




if __name__ == "__main__":
    main(sys.argv[1:])
