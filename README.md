# SoftEng-306-Project Group 8 (FloppyDisk)
A simulation tool for testing harvesting robots in kiwifruit orchards

##Prerequisites

* Ubuntu 14.04
* ROS Indigo

##Getting the Software

git clone 

		https://github.com/Ofekw/SoftEng-306-Project-1.git

##Configuring the application (optional)
  In the root directory you will find a "config.properties".Open it with any text editor and edit only the values (do not      change the variable names).
  
  Anything that is updated in here will be generated and updated when calling the run.py script.


##Running the program 

1. In terminal navigate to the home directory

		cd se306-project-1
	
2. Run the project

	 	./run.py


##Troubleshooting 
Update	ROS_PACKAGE_PATH

* in terminal call

		gedit ~/.bashrc

* Scroll to the bottom of the file
* Add 

		source ~/<path to workspace>/setup.bash (Leave the other source commands)
		
* Update the export to 

		export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/<path to workspace>/se306Project1
