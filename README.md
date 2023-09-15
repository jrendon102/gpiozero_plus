# yb_expansion_board

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A Python Library for controlling the Yahboom 4WD expansion board with Raspberry Pi.

This library provides a Python interface for controlling various functions and components on the Yahboom 4WD expansion board when used with a Raspberry Pi.


## Dependencies

This library relies on the following dependencies:

- **gpiozero**: A Python library for controlling GPIO pins on Raspberry Pi and other supported platforms. It is used to interface with the hardware components on the Yahboom expansion boards.

- **pigpio**: A library for interfacing with the GPIO pins on the Raspberry Pi. It is required for certain functionality in this library.


## Installation

   You can install the **yb expansion board** library using  `pip`. If you haven't already, make sure you have Python and pip installed on your system.

   1. **Clone the Repository**: Clone the **yb expansion board** repository to your local machine.

      ```bash
      git clone https://github.com/jrendon102/yb_expansion_board.git
      ```

   2. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
      
      ```bash
      cd yb_expansion_board
      ```

   3. **Install the Package**: Use `pip` to install the package after building it using the provided setup script.

      First, build the package:

      ```bash
      python3 setup.py sdist
      ```

      Next, use `pip` to install the package from the generated distribution:
      
      ```
      pip install .
      ```
      
      Finally, verify that the package was installed successfully.
      
      ```bash
      pip list | grep yb_expansion_board
      ```

To install **pigpio** library follow these steps:

   1. Navigate to `opt/` directory
      
      ```bash
      cd /opt
      ```
   
   2. Download the lastes version of pigpio:

      ```bash
      sudo wget https://github.com/joan2937/pigpio/archive/master.zip
      ``` 
   
   3. Unzip the downloaded archive and remove it:
     
     ```bash
     sudo unzip master.zip && sudo rm master.zip
     ```

   4. Change to the pigpio-master direcotry:

      ```bash
      cd pigpio-master
      ```

   5. Build and install pigpio:
   
      ```bash
      sudo make
      sudo make install
      ```

After completing these steps, you should have pigpio installed and ready for use with the Yahboom 4WD expansion board.

For more information about pigpio, you can visit the [official website](http://abyz.me.uk/rpi/pigpio/). Information about the pigpio Daemon can be found in the [official website](http://abyz.me.uk/rpi/pigpio/pigpiod.html).

## Uninstall
You can uninstall the **yb_expansion_board** using the following command:

   ```bash
   pip uninstall yb_expansion_board
   ```

## Example
Here's an example of how to use the `yb_expansion_board` library in your Python project:

1. Start the pigpio daemon by running the following command:
   
   ```bash
   sudo pigpiod
   ```

2. Create a script called `my_fan.py` with the following code:
   
   ```python
   #!/usr/bin/env python3
   import time
   from yb_expansion_board.ybFan import YBFan

   # Initialize and use the various components from the library
   fan = YBFan(fan_pin=2)

   try:
       # Turn fan on for 5 seconds
       print("Turning fan on for 5 seconds")
       fan.control_fan(fan_enable=True)
       time.sleep(5)

       # Turn fan on off 5 seconds
       print("Turning fan off for 5 seconds")
       fan.control_fan(fan_enable=False)
       time.sleep(5)

   # Catch Keyboard interrupt to prevent motor from continuing to
   # operate if 5 seconds have not elapsed.
   except KeyboardInterrupt:
       print("Keyboard interrupt received.")
   fan.disconnect()
   ```

3. Finally run the command `python3 my_fan.py` to see the fan in action.
## More Information
For more information about the Yahboom 4WD expansion board, including schematics, user guides and GPIO pin numbering, please visit the [officiall website](http://www.yahboom.net/study/4wd-ban).

## License

This project is licensed under the [MIT License](LICENSE) - see the [LICENSE](LICENSE) file for details. 

## Author & Maintainer
Julian Rendon

Email: julianrendon514@gmail.com