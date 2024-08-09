# Heimdall Launch Files:

## How To Run a Launch File

After successfully building the workspace, launch files can now be accessed and executed.

To run the launch files, navigate to the root of the workspace and run the following commands:
```bash
source install/setup.bash
```
Now you can execute the following launch files using the following command:
```bash
ros2 launch launches/<launch_category>/<launch_file_name>
```

The possible launch categories are:
- `base_station`  → Launch Files to be run on a Base Station Computer
- `heimdall`      → Launch Files to be run on Heimdall
- `wanderer2`     → Launch Files to be run on wanderer2
- `testing`       → Launch Files created to be used for temporary testing
- `components`    → Launch Files that are used for general purposes

## Current Launch Files 

Below are each of the currently existing launch files. The specific functionality of the launch files, details about the launch files such as parameters, arguments, and configuration .yaml files are included for each of the launch files. 

If you are preparing another launch file for the rover, please be sure to include a similar level of detail about the functionality and use of the new launch file you are making.

- `drivemotor_testing_gamepad.launch.py`
  - This file launches the node responsible for the drivebase motors and a joystick controller for controlling the drivebase 

- `joy.launch.py`
  - This file launches the joystick used to control the rover
  - The launch file takes in a parameter called "joy_config". The joy config is what allows the buttons on the controller to map to different functionalities.
  - By default, the "8bit" controller is used, but can be changed to the following values:
    - "8bit"
    - "f710"
    - "ps5"

- `lidar.launch.py`
  - This file launches the velodyne lidar
  - To change the configuration of the lidar, edit the [.config.yaml](config/velodyne.config.yaml) file found under the config directory. Things like the device_ip, or rpm may need to be changed. 
