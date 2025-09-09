Project for [robotics dojo 2024](https://roboticsdojo.github.io/competition2024.html). A few mods to Josh Newans' articubot_one project documented on [github](https://github.com/joshnewans/articubot_one) and [youtube](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT). 

# Quick Links
* [Sources](#sources)
* [Hardware Configuration](#hardware-configuration)
* [Software Dependencies](#software-dependencies)
* [Modified Parameters](#modified-parameters)
    * [Note](#note)
* [Usage](#usage)
    * [Prerequisites](#preliminary)
    * [Simulation](#simulation)
    * [Real Robot](#real-robot)
    
* [Misc.](#misc)
* [Future](#future)

# Sources
- Simulation map obtained from [Brian Macharia's repo](https://github.com/ru3ll/dojo/blob/main/worlds/dojo2024). Slightly modified to remove one wall which was not present in the real gamefield
- Map evaluation script obtained from [Dr. Shohei Aoki's repo](https://github.com/shohei/Map_Evaluation)
- diffDriveArduino from [RedstoneGithub's fork](https://github.com/RedstoneGithub/diffdrive_arduino) of Josh's diffdrive arduino
- [sllidar](https://github.com/Slamtec/sllidar_ros2) and [rplidar](https://docs.ros.org/en/ros2_packages/humble/api/rplidar_ros/) from those links
- [joy tester](https://github.com/joshnewans/joy_tester) is from Josh
- [serial](https://github.com/joshnewans/serial) is from Josh
- [serial_motor_demo](https://github.com/joshnewans/serial_motor_demo) is from Josh
- [rosArduinoBridge](https://github.com/joshnewans/ros_arduino_bridge) is Josh's fork of [hrobotics work](https://github.com/hbrobotics/ros_arduino_bridge)


Our [technical design paper](Technical_Design_Paper_Joint_Team_3_Knights_and_Pentagon.pdf) and [technical presentation slides](robotics-dojo-2024-knights-pentagon-presentation.pdf) may have a few more details on this project.

# Hardware Configuration
- Raspberry Pi running Ubuntu 22.04, dev machine running Ubuntu 22.04
- RPLidar A1 M8 connected to Pi via USB
- Arduino Mega running [ROSArduinoBridge.ino](arduino-code/ROSArduinoBridge/ROSArduinoBridge.ino), connected to Pi USB
- 200 RPM motors with built-in encoders, with the encoders connected as shown in [encoder_driver.h](arduino-code/ROSArduinoBridge/encoder_driver.h)
- L298N motor driver, with motor controls connected as shown in [motor_driver.h](arduino-code/ROSArduinoBridge/motor_driver.h)

Here's the bot [here](our-robot1.png), [here](our-robot2.png)


# Software Dependencies
- Ubuntu 22.04 - ROS2 Humble pair was used here. Docker on windows is not recommended by us because of network woes. The WSL network is isolated from the system network, so ROS on Pi does not communicate with ROS on Docker by default. We were not able to get it working in time. Apart from that, native Ubuntu has way higher performance.
- twist-mux, ros-humble-teleop-twist-keyboard, ros-humble-teleop-twist-joy, slam-toolbox, ros-humble-navigation2, ros-humble-nav2-bringup, ros-humble-gazebo-ros-pkgs, ros-humble-xacro, ros2 control, [rplidar ros](https://docs.ros.org/en/humble/p/rplidar_ros/)
<!-- - ros-humble-gazebo on pc -->
- tmux (or similar) is recommended because you'll need to have multiple terminals at any given moment


# Modified Parameters
## src/jkl/description
### robot_core.xacro and robot.urdf.xacro
It's mostly Josh's code only that some elements are removed and others are modified

### gazebo_control.xacro
```htm
<wheel_separation>0.275</wheel_separation>
<wheel_diameter>0.085</wheel_diameter>
```

### ros2_control.xacro
```xml
<!-- arduino mega -->
<param name="device">/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_12148509806232150650-if00</param> 
<param name="baud_rate">57600</param>
<param name="enc_counts_per_rev">988</param>
```


## src/jkl/config/
### mapper_params_online_async.yaml
```yaml
resolution: 0.025 # (0.05 is default) 
loop_search_maximum_distance: 6.0 # (default is 3.0). The default value causes the robot to "teleport" during mapping, ruining a good map
```

### nav2_params.yaml
- initial_pose (not defined by default). Helps save time when using localization. A new pose can be given using rviz. Use it by having the odom reset (relaunching launch_robot) then launching online_async.launch.py. Then set the initial pose as 0 0 0
- controller_server: [mppi controller server](https://docs.nav2.org/configuration/packages/configuring-mppic.html) was used (dwb is default). rpp, teb and grace might have worked better, but mppi worked best out of the box
- local_costmap & global_costmap: resolution: 0.025 (0.05 is default). Seems to get robot get stuck less frequently
- local_costmap & global_costmap: robot_radius: 0.025 (0.05 is default). Seems to get robot get stuck less frequently
- amcl laser_max_range and min_range 12.0 and 0.3 respectively (default is )
- velocity smoother feedback: "CLOSED_LOOP" (default is OPEN_LOOP)
- velocity smoother velocities and accelerations: [0.35, 0.0, -0] and [12.5, 0.0, 8.2] (default are something else) The accelerations are very large. Something better should be identified and used

### my_controllers.yaml
```yaml
wheel_separation: 0.275
wheel_radius: 0.0425
```
Explore the other params in this file

## src/jkl/launch/
### rplidar.launch.py
```json 
"serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
```

## all files in src/sllidar_ros2/launch and src/rplidar_ros/launch
```python 
serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0') # rplidar
```

## Note
- Josh goes over how to obtain wheel diameter, wheel separation and encoder counts per revolution in [this video](https://www.youtube.com/watch?v=4VVrTCnxvSw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)
- **online_async_launch.py** **localization_launch.py**, **navigation_launch.py**, **mapper_params_online_async.yaml**, **nav2_params.yaml** were copied from the default slam-toolbox and nav2_bringup directories (/opt/ros/humble/share/nav2-or slam-toolbox...) then slightly modified (for example in localization_launch.py instead of `get_package_share_directory('nav2_bringup')` use `get_package_share_directory('jkl')`) This is the recommended way to do it because, Josh's articubot_one repo uses ROS foxy. Apart from fewer parameters in nav2 and mapper yaml, some problems might come up like failing to launch navigation. Some elements were renamed in ROS Humble, like recoveries->behaviors
- all instances of serial ports use `dev/serial/by-id` path. This was preferred over `dev/serial/by-path` and `dev/tty*` because it depends on the device ID rather than the port used or connection order

# Usage
## 
- Wifi (or ethernet) has to be connected, on both pi and PC whenever you're running ROS, especially if using cyclone DDS as the RMW_IMPLEMENTATION
## Preliminary
- [install ros-humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- add `source /opt/ros/humble/setup.bash` to ~/.bashrc so you don't have to in every new terminal
- install [nav2, slam-toolbox](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/), colcon, and optionally [change RMW_IMPLEMENTATION to cyclone_dds](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)
- install controllers, plugins, xacro and twist mux 
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers && \
# sudo apt install ros-humble-gazebo-ros2-control && \ # Gazebo Classic went EOL so this won't work, make migration
# sudo apt install ros-humble-gazebo-ros-pkgs && \ 
sudo apt install ros-humble-xacro && \
sudo apt install ros-humble-twist-mux && \
sudo apt install ros-humble-slam-toolbox && \
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup && \
curl -sSL http://get.gazebosim.org | sh && \
sudo apt install ros-humble-gazebo-ros
```
- disable brltty service. This service always takes serial port devices to be braille device, preventing you from accessing your microcontroller. Disable the service
```bash 
systemctl stop brltty-udev.service && \
sudo systemctl mask brltty-udev.service && \
systemctl stop brltty.service && \
systemctl disable brltty.service
```
- enable read/write to serial port, you can do this by adding yourself to the dialout group `sudo usermod -a -G dialout $USER` **then reboot**
- clone the repository in both Pi and dev machine
- cd to the location of src directory in both Pi and dev machine e.g. `cd ~/robotics-dojo-2024/pi-code`
- edit pi-code/src/jkl/description/ros2_control.xacro line 11 to the appropriate for your microcontroller. You can check which is appropriate by connecting the microcontroller to your machine then running `ls /dev/serial/by-id`. You can then add it as appropriate
- run `colcon build --symlink-install` in both Pi and dev machine. It may throw a lot of errors and/or warnings on first build, just ignore and run `colcon build --symlink-install` again. This should be fixed in future versions of this project.
- run `source install/setup.bash` on each terminal (or add `source yourworkspace/install/setup.bash` to ~/.bashrc) in both Pi and dev machine

## Simulation
Everything is run on pc. Instead of running `ros2 launch jkl launch_robot.launch.py` **and** `ros2 launch jkl rplidar.launch.py`, use `ros2 launch jkl launch_sim.launch.py use_sim_time:=true world:=./src/jkl/worlds/dojo2024` 
Change every instance of `use_sim_time:=false` to `use_sim_time:=true`
All else is as below

## Real Robot
- cd to the location of src directory in both Pi and dev machine e.g. `cd ~/robotics-dojo-2024/pi-code`
- On PC run `rviz2 -d ./src/comp.rviz`
- On Pi, run `ros2 launch jkl launch_robot.launch.py`
- On Pi in another terminal run `ros2 launch jkl rplidar.launch.py`. You may also use `ros2 launch rplidar_ros rplidar_a1_launch.py` or `ros2 launch sllidar_ros2 sllidar_a1_launch.py`. In a few tests, we noticed that the latter two run at 8kHz, instead of the former's 2kHz. Also, these launch files show the lidar is in `Sensitivity` mode rather than `Standard` mode. We're not sure if there are benefits in using these two over Josh's. 

### For mapping phase
- On PC (Pi is also fine but during competition we found PC was better, did not have the "teleportation" issue) run `ros2 launch jkl online_async_launch.py slam_params_file:=./src/jkl/config/mapper_params_online_async.yaml use_sim_time:=false`
- On PC or Pi run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- you may also use gamepad instead of keyboard, but you'll have to configure the joystick yaml appropriately. Josh demonstrates the process well [here](https://www.youtube.com/watch?v=F5XlNiCKbrY&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=17). But in short run `ros2 run joy joy_node` in one terminal then `ros2 run joy_tester test_joy` in another. Press a key, identify its id using the GUI and use that id in the jkl/config/joystick.yaml. Having configured the yaml correctly, you can use `ros2 launch jkl joystick.launch.py` in place of or in conjuction with teleop_twist_keyboard. joystick **does not** require its terminal window to be in focus to work
- With the teleop_twist_keyboard terminal in focus use `u i o j k l m , .` to drive the bot around until a satisfactory map is shown in rviz
- Open slam toolbox panel in rviz and fill the input boxes with a desired name. Click save map and serialize map. Four map files will be saved in current directory

### For navigation phase
- (on pc or pi, but pc is more powerful) You may use slam_toolbox or localization_launch for localization. slam_toolbox is not recommended since it (currently) overwrites (or updates) the existing map with new scans. If you wish to proceed anyway, this is how you do it: change `mode: mapping` to `mode: localization` and uncomment `map_file_name:` followed by path to the **serialized** map file (i.e. the file that has .data or .posegraph extension), without extension, in src/jkl/config/mapper_params_online_async.yaml. Then run same command as when mapping `ros2 launch jkl online_async_launch.py slam_params_file:=./src/jkl/config/mapper_params_online_async.yaml`.
- To use localization_launch, kill online_async if it was running then run  `ros2 launch jkl localization_launch.py map:=path_to_map.yaml use_sim_time:=false`. To set/change pose estimate, you can use rviz
- (on pc or pi, but pc is recommended because of computation limitations on pi) run `ros2 launch jkl navigation_launch.py map_subscribe_transient_local:=true params_file:=./src/jkl/config/nav2_params.yaml use_sim_time:=false`
- Use rviz to set waypoints either one by one or all points at once. Navigate through poses creates one path across all waypoints while navigate to waypoints moves the robot to each point one-by-one

# Misc.
- the lidar sometimes acts up when attempting to launch. If it fails to launch even after mutliple retries, (1) try **unplug and replug** connections, i.e., usb connection on pi, usb on lidar, the lidar daughter board cables, other daughter board cable motor side and laser side, you get the idea (2) try using a different type-c cable, or different power supply altogether for the raspberry pi. 
- you can save changes to comp.rviz (or the default rviz config) when you hit Ctrl+S so that you don't have to make the same changes when relaunching rviz
- for simulations, slow computers may produce unrealistic navigation behaviours, like aborting too soon. Gazebo is a resource hog. So try to launch launch_sim in one pc then everything else in another pc configured in the same way, connected in the same network 
- use float where float is used in yaml files, otherwise stuff may fail to launch
- sometimes things don't work very well, relaunch and reboot are a must
- rplidar.launch.py sometimes takes a few tries to get working, also try disconnecting arduino then launch lidar first
- use `export ROS_DOMAIN_ID=some-number-that-is-not-the-default-0` per terminal or in bashrc to avoid communicating with other people's ROS environments
- cyclonedds is recommended over fastrtps because ["it doesn't work well with nav2"](https://roboticsbackend.com/ros2-nav2-tutorial/). Install using `sudo apt install ros-humble-rmw-cyclonedds-cpp` and then per terminal or in bashrc `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Raspberry running ubuntu 22.04 sometimes takes a long time to boot. Using `systemd-analyze blame`, it seems to be caused by `systemd-networkd-wait-online.service`. A workaround is turning `optional` to `false` in `/etc/netplan/50-cloud-init.yaml`. A better fix should be identified. 

# Future
- Fix weaving. When the path is relatively straight the robot follows a winding path, it weaves left and right, which might cause it to hit an obstacle. In the competition, we reduced the maximum angular velocity to work around this.
- Make the robot smaller (particularly wheel_separation), like [Atom x RoboQueens robot](atom-roboqueens-robot.png)
- Isolate the arduino and motor driver, tristate buffer and/or relay?
- Find a way to power lidar motor properly. Why was the USB hub buzzing when switched on? Modify a USB cable, so that an additional power cable is soldered to the 5V line, for simplicity and compactness
- Get a really good USB PD power bank for the Raspberry Pi
- Get USB-C trigger boards, or build battery pack with BMS to avoid overdischarging motor power supply like last time
- Wifi sucks. Check if you can get a better AP or ditch wifi altogether and use thin ethernet cable
- Try increasing rates (like decreasing map update interval, increase controller frequency, map publish rates, robot velocities etc) in mapper params and nav2 params
- Try [RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html#), TEB, [Graceful](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html) and [DWB](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html) controller servers (with and without [rotation shim](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html))
- Restore the original ISR in ROSArduinoBridge using the newfound knowledge in register operations. Should have a small speed boost.
- Try using a differert fork of supporting packages (like serial and serial motor demo) because the ones used have easy_install deprecation warnings during colcon build --symlink-install
- Try using very high resolution laser scan and local and global costmaps to see if the bot ever gets stuck (without modifying robot radius that is)
- Try a different path planner, like [smac](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html) or [thetastar](https://docs.nav2.org/configuration/packages/configuring-thetastar.html), maybe they will perform better with higher resolution costmaps. When tested with resolution 0.05, generated paths were straighter and often passing through the robot radius regions hence the robot would get stuck in recovery easily. Also explore effect of deadband velocity, as well as other params n this file.
- Try and use `ros2 launch sllidar_ros2 sllidar_a1_launch.py` or `ros2 launch rplidar_ros rplidar_a1_launch.py` instead of jkl rplidar.launch.py to see if the 8kHz and 'Sensitivity' vs 2kHz and 'Standard' makes it better
- Try using a different bt_navigator xml such as the ones in jkl/config/behavior_trees. You specify the one to use in nav2_params.yaml
- Explore effect of other params in my_controllers.yaml. There's also closed-loop there, might be valuable
- Make the mapping phase also autonomous
- Incorporate IMU, maybe this has benefits
- Try depth camera instead of lidar
- Make a simple BMS, can add a simple MOSFET switch to prevent overdischarge from battery. Can also configure battery pack to be able to charge using LiPo charger, something like 3s 2p config?
- a guest on Tech Expo said we should check AWS Deepracer 
- Maybe explore [Ackermann](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) steering? :)

