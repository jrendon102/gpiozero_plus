# Expansion Board Driver

### Overview

ROS package that is used to communicate with the [Yahboom G1 Tank](http://www.yahboom.net/study/G1-T-PI) expansion board.

### Dependencies

1. gpiozero

   - Download and installation can be found [here](https://gpiozero.readthedocs.io/en/stable/installing.html).

2. rplidar package

   - You can install rplidar package by running the following command:
     ```
     sudo apt install ros-noetic-rplidar-ros
     ```
   - Information regarding rplidar package can be found [here](http://wiki.ros.org/rplidar).

3. hector_slam_launch package
   - You can install hector_slam package by running the following command:
     ```
     sudo apt install ros-noetic-hector_slam_launch
     ```
   - Information regarding hector_slam_launch package can be found [here](http://wiki.ros.org/hector_slam_launch).
   - Detailed information regarding hector slam can be found [here](http://wiki.ros.org/hector_slam).

## Setup

1. Start by creating a ROS workspace and navigate to your src/ directory. Run the following to clone the directory inside your src/ directory.

   ```
   git clone git@github.com:jrendon102/expansion_board_driver.git
   ```

2. Now build and source your workspace.

3. <mark>Important:</mark> Make sure to run the following commands before doing anything. This will launch the pigpio library which is necessary to communicate with the hardware.

   ```
   sudo pigpiod
   ```

   - <mark>NOTE:</mark> If using RPLiDAR A1 make sure to do the following:

     - Check the authority of rplidar's serial-port :

       ```
       ls -l /dev |grep ttyUSB
       ```

     - Add the authority of write: (such as /dev/ttyUSB0)
       ```
       sudo chmod 666 /dev/ttyUSB0
       ```

4. Now you are able to start up and communicate with specific hardware by running the following launch file.

   ```
   roslaunch expansion_board_driver setup_hardware.launch
   ```

5. You should now be able to communicate with various hardware attached to the expansion board.

### Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)
