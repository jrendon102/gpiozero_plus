# Expansion Board Driver

### Overview
ROS package that is used to communicate with the [Yahboom G1 Tank](http://www.yahboom.net/study/G1-T-PI) expansion board.

### Dependencies
gpiozero
- Download and installation can be found [here](https://gpiozero.readthedocs.io/en/stable/installing.html).

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

4. Now you are able to start up and communicate with specific hardware by running the following launch file.
   ```
   roslaunch expansion_board_driver setup_hardware.launch
   ``` 

5. You should now be able to communicate with various hardware attached to the expansion board.

### Example
1. After setting up your ROS workspace, building and sourcing, run the following command.
   ```
   roslaunch expansion_board_driver setup_hardware.launch
   ```
   This will allow communication with the motors.

2. Next, open a new terminal, source your workspace and run the following command.
   ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.15 _turn:=0.17 _key_timeout:=0.5
   ```
    This will allow you to move the robot within a safe speed by using the teleop_twist_keyboard package. More info about this package can be found [here](http://wiki.ros.org/teleop_twist_keyboard).

### Author & Maintainer
Julian Rendon (julianrendon514@gmail.com)