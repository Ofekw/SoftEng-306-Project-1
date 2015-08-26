import subprocess
import sys
import time
from os import listdir
from os.path import isfile, join, devnull
import time
import generateWorldFile

config = {}
def main(argv):

    global verbose_mode
    global log
    global error_log
    global total_tests
    global total_errors

    total_tests = 0
    total_errors = 0

    print_header()

    verbose_mode = False
    if len(argv) > 0:
        if argv[0] == "-v":
            verbose_mode = True

    with open("config.properties", "r") as f:
        for line in f:
            property = line.split('=')
            config[property[0]] = property[1]
    processes = []
    mypath = 'se306Project1/test'
    test_files = []
    for f in listdir(mypath):
        if isfile(join(mypath,f)):
            if f.startswith('Test_'):
                test_files.append(f)

    #logging
    log = open('test.log' ,'w+')
    error_log = open('test_error.log' ,'w+')

    #build and generation test
    test_build()

    #start ros services
    start_services(processes)

    #generate entities
    setup(processes)

    #run unit tests
    run_tests(processes, test_files)

    #Print final outcome of all tests to screen
    print_final_outcome()

def test_build():
    print "Build Script Testing:\n"
    test_file = 'TestGenerateFiles.py'
    print("---------TESTING: " + test_file +'\n')
    s = subprocess.call(["chmod","+x",'./'+test_file])
    p = subprocess.Popen(['python', test_file], shell=False, stdout=log, stderr=error_log)
    p.wait()
    if verbose_mode:
        print_test_summary_verbose()
    else:
        print_test_summary_short()

def start_services(processes):
    generateWorldFile.main(config)
    FNULL = open(devnull, 'w')
    processes.append(subprocess.Popen(['roscore'], shell=False, stdout=FNULL, stderr=error_log))

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
        print("---------TESTING: " + file +'\n')
        s = subprocess.call(["chmod","+x",'se306Project1/test/'+file])
        p = subprocess.Popen(['rosrun', 'se306Project1', file], shell=False, stdout=log, stderr=error_log)
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

        if verbose_mode:
            print_test_summary_verbose()
        else:
            print_test_summary_short()

    cleanup(processes)


def print_test_summary_verbose():
    global total_tests
    global total_errors
    line_list = (open('test.log').readlines())
    line_list.reverse()
    for i in range(len(line_list)):
        if "SUMMARY" in line_list[i]:
            print "\t\t"+red_string(line_list[i-1])
            print "\t\t"+line_list[i-2]
            print "\t\t"+line_list[i-3]
            print "\t\t"+line_list[i-4]
            test_count = int(line_list[i-2].split()[2])
            total_tests += test_count
            error_count = int(line_list[i-3].split()[2])
            failure_count = int(line_list[i-4].split()[2])
            total_errors += (error_count+failure_count)
            break

def print_test_summary_short():
    global total_tests
    global total_errors
    line_list = (open('test.log').readlines())
    line_list.reverse()
    for i in range(len(line_list)):
        if "SUMMARY" in line_list[i]:
            outcome = green_string("SUCCESS") if "SUCCESS" in line_list[i-1] else red_string("FAILURE")
            test_count = int(line_list[i-2].split()[2])
            total_tests += test_count
            error_count = int(line_list[i-3].split()[2])
            failure_count = int(line_list[i-4].split()[2])
            total_errors += (error_count+failure_count)
            passed_count = test_count - error_count - failure_count
            print "\t\t"+ outcome + " (" +str(passed_count)+"/"+str(test_count) + ")\n"
            break

def print_final_outcome():
    print "\n_____________________________________________________\n"

    print final_outcome_msg()
    print "_____________________________________________________\n"


def print_header():
    print "\n_____________________________________________________\n"

    print "                Floppy Disk Testing Unit"
    print "_____________________________________________________\n"

def final_outcome_msg():
    string = "                Result:" + str(total_tests-total_errors)+"/"+str(total_tests)
    percentage = (float(total_tests)-float(total_errors))/float(total_tests)
    if percentage == 1:
        return green_string(string)
    elif percentage >= 0.75:
        return yellow_string(string)
    else:
        return red_string(string)

def red_string(string):
    return "\033[31m"+string+"\033[0m"

def green_string(string):
    return "\033[32m"+string+"\033[0m"

def yellow_string(string):
    return "\033[33m"+string+"\033[0m"

def spinning_cursor():
    while True:
        for cursor in '|/-\\':
            yield cursor


if __name__ == "__main__":
    main(sys.argv[1:])
