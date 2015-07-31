# SoftEng-306-Project Group 8 (FloppyDisk)
A simulation tool for testing harvesting robots in kiwifruit orchards

Install instructions. 

1)Create a file named .#setupConfig in your home directory.
2)Add the path to your workspace containing the se306Project1 package as the first line of the .#setupConfig file.
  e.g default folder should is (if your git repo is in your home directory): SoftEng-306-Project-1/

Run Instructions.

Run build script.

From commandline in your workspace run:
python run.py

Adding Robots.

A robotList.txt file has been added at the project root directory. To add a new robot, simply add <robotsName>.py on a new line in the file.

Update ROS_PACKAGE_PATH

Run: gedit ~/.bashrc
Scroll to the bottom of the file.
Add: source ~/<path to workspace>/setup.bash (Leave the other source commands)
Update the export to: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/<path to workspace>/se306Project1
