# SoftEng-306-Project Group 8 (FloppyDisk)
A simulation tool for testing harvesting robots in kiwifruit orchards

###Prerequisites
It is recommended to run the simulator on Ubuntu 14.04 using [ROS Indigo](http://wiki.ros.org/indigo) and Python.

Click [here](http://wiki.ros.org/indigo/Installation/Ubuntu) for instruction on installing ROS Indigo 

###Getting the Software
 Clone the repo from GitHub.

         $ git clone https://github.com/Ofekw/SoftEng-306-Project-1.git

###Running the program 
1. In terminal navigate to the home directory

		cd SoftEng-306-Project-1
	
2. Run the project

	 	./run.py

[Click here for more information about the launcher](https://github.com/Ofekw/SoftEng-306-Project-1/wiki/3.1.1-Run-script)
###Troubleshooting 
Update	ROS_PACKAGE_PATH

* in terminal call

		gedit ~/.bashrc

* Scroll to the bottom of the file
* Add 

		source ~/<path to workspace>/setup.bash (Leave the other source commands)
		
* Update the export to 

		export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/<path to workspace>/se306Project1
