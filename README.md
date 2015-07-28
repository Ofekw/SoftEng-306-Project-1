# SoftEng-306-Project Group 8
A simulation tool for testing harvesting robots in kiwifruit orchards

Install instructions. 

1)Create a file named .#setupConfig in your home directory.
2)Add the path to your workspace containing the se306Project1 package as the first line of the .#setupConfig file.
  e.g default folder should is (if your git repo is in your home directory): SoftEng-306-Project-1/

Run Instructions.

Note: # = run command in new terminal. Start with terminal in workspace containing se306Project1 etc.

1)#Roscore
2)#Rosrun stage_ros stageros world/myworld.world
3)Cd se306Project1
4)#Rosmake
5)Cd ..
6)Rosrun se306Project1 R0
7)#Rosrun se306Project1 R1

Update ROS_PACKAGE_PATH

Run: gedit ~/.bashrc
Scroll to the bottom of the file.
Add: source ~/<path to workspace>/setup.bash (Leave the other source commands)
Update the export to: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/<path to workspace>/se306Project1
