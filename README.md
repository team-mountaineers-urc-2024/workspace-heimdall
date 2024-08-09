# <font size="+12">Workspace-Heimdall</font>
***This is the main repository for WVU's URC Heimdall Codebase***

# Table of Contents
[Installation](#installation)  
[Troubleshooting](#troubleshooting)  

# Installation
Ubuntu 22.04 is required as our codebase runs off ROS 2 Humble.  
Throughout this guide, "$" denotes commands to run in the terminal.
### Pre-Install: SSH Key  
An ssh key for your current machine must be added to your GitHub account to clone our repositories.  
SSH keys are used for passwordless authentication.  
For more information see https://docs.github.com/en/authentication/connecting-to-github-with-ssh.

* #### Creating an SSH Key
	`$ ssh-keygen`  
	Keep the default file name, add a password if you'd like.  

* #### Adding the SSH key to your Github account
	Go to https://github.com/settings/keys and click "New SSH Key"  
	Title can be anything  
	For Key, copy-paste the output of `$ cat ~/.ssh/id_rsa.pub`  

### Clone workspace-heimdall to ~/workspace-heimdall
Install git if not done so already with `$ sudo apt install git` then:  
`$ git clone git@github.com:wvu-urc/workspace-heimdall.git ~/workspace-heimdall`  
* If this is your first time talking to GitHub on your machine, you will receive a warning saying authenticity cannot be established.  
	Type "yes" and hit enter.

### Install ROS 2 Humble
The script `install_humble.bash` located in the same directory as this file will install ROS 2 Humble onto the current machine.  
It is based on [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with additions for WVU URC specific stuff. 
It is recommended to be familiar with the install guide linked above, as well as WVU URC specific steps in the script, before running it.  
It assumes the current machine is running __Ubuntu 20.04__. It is the only recommended OS for WVU URC.  
The script also assumes you are using __bash__ (default).

* #### Run the Install script
	`$ cd workspace-heimdall`  
	`$ ./setup/install_humble.bash`  
	The script will take roughly 10-30 minutes to complete (It installs ~1000 packages among other things)  
	Once the script finishes, it will print out a message saying so.  
	* Please take a moment to read through the script log to see if any errors occurred. If an error did occur, see [Troubleshooting](#troubleshooting).  
	* __After the script finishes, run `$ source ~/.bashrc`__


* #### Clone the other repositories
	`$ vcs import src < repos.yaml`
	Warning: This command can rarely fail due to Github's anti-spam protection or poor internet connection (like in ERB 115). If this happens then wait a few minutes and try again. Sometimes it may be required to run `$ git init` if waiting does not fix the problem.

* #### Install URC Dependencies
	In addition to the base ROS 2 Humble installed above, additional dependencies are needed for URC. These can be installed using rosdep with the command below. Make sure you're still in the workspace-heimdall directory.  
	`$ rosdep install --from-paths src --ignore-src -r -y`

[//]: < This will eventually be added back in, but it is broken until further notice. Refer to Jalen, Jon, and Emily who were present for the ROS Wanderer II fix >
[//]: < * #### Install Pixhawk Geographic libs>
	[//]: < You'll also need some geographic libraries in order to use the Pixhawk FCU, they can be installed by running the command below to download and run the installer script. > 
	[//]: < `$ sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O - | sudo bash`>

* #### Compile the workspace
	While still in workspace-heimdall, run  
	`$ colcon build`
	See [Troubleshooting](#troubleshooting) if you receive errors or crashes. 

* #### Utilize the workspace
	To launch our different launch files on the rover, please reference the [README.md](launches/README.md) in the "launches" directory.
	
* #### Post Install Notes
	* To pull from all repos in src at once, run `$ vcs pull src`  
	* Make sure to install any new dependencies after pulling new commits using the command from "Install URC Dependencies"
	* Remember to recompile after any code change or commit pull (other than Python or launch files)
	* Check out the official ROS 2 docs for tutorials, examples, and information on how ROS 2 works!  [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)  
	* Don't be discouraged if you accidentally break your Humble install. Everyone does at some point. It's all a part of the learning process.
	* Visual Studio Code is a good editor you can install with `$ snap install code --classic`


# Troubleshooting

* CMake Errors: 
  * The current CMakeCache.txt  &lt;directory&gt; is different than the directory &lt;directory&gt; where CMakeCache.txt was created. This may result in binaries being created in the wrong place.
  * The source directory &lt;directory&gt; does not exist
  * For both of these errors, run `$ rm -rf ./log ./install ./build`.
  * This command removes the built colcon packages. After removing them, build the workspace like you normally would.
* I'm getting missing dependency errors while compiling, but I know those dependencies are there!  
	* This can sometimes happen due to dependency race conditions, where project A relies on project B, but project A tries to compile before project B is done. 
	* Try running `$ colcon build --continue-on-error` twice. This allows cmake to keep building other projects even if one project fails due to an error. The first time will build the dependencies that keep being missed, and the second time should compile the rest of the workspace.
* My computer/VM keeps crashing when I try to build the code
    * By default Colcon tries to compile 4 packages at once. When working with larger packages (specifically C++) this can tax the CPU enough to cause a crash
    * Try running `$ colcon build --executor sequential`. This will force colcon to build packages one at a time. If this does not fix the issue, keep restarting your machine and building with the tag again. The compiler will save its stopping point in the event of an unexpected crash so no progress should be lost.
