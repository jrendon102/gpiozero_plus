# gpiozero_plus

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A Python Library providing enhanced features and utilities for gpiozero.

This library is a Python wrapper designed to simplify the setup and control of components using the gpiozero library. It provides a convenient interface for managing various hardware components when working with Raspberry Pi, making it easier to interact with GPIO pins. It is a versatile tool for anyone looking to streamline the control of hardware components in Python applications.


## Dependencies

This library relies on the following dependencies:

- **gpiozero**: A Python library for controlling GPIO pins on Raspberry Pi and other supported platforms.

- **pigpio**: A library for interfacing with the GPIO pins on the Raspberry Pi. It is required for certain functionality in this library.


## Installation

   You can install the **gpiozero plus** library using  `pip`. If you haven't already, make sure you have Python and pip installed on your system.

   1. **Clone the Repository**: Clone the **gpiozero plus** repository to your local machine.

      ```bash
      git clone https://github.com/jrendon102/gpiozero_plus.git
      ```

   2. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
      
      ```bash
      cd gpiozero_plus
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
      pip list | grep gpiozero_plus
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
   3. Unzip the downloaded archive and remove it
      
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

After completing these steps, you should have pigpio installed and ready for use.

## Uninstall
You can uninstall the **gpiozero_plus** using the following command:

   ```bash
   pip uninstall gpiozero_plus
   ```

## Example
Here's an example of how to use the `gpiozero_plus` library in your Python project:

1. Start the pigpio daemon by running the following command:
   
   ```bash
   sudo pigpiod
   ```

2. Create a script called `my_fan.py` with the following code:
   
   ```python
   #!/usr/bin/env python3
   import time
   from gpiozero_plus import Fan

   # Initialize and use the various components from the library
   fan = Fan(fan_pin=2)

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
   
   # Make sure to release the GPIO pin(s)
   fan.disconnect()
   ```

3. Finally run the command `python3 my_fan.py` to see the fan in action.
## More Information
For more information about pigpio, you can visit the [official website](http://abyz.me.uk/rpi/pigpio/). 

Information about the pigpio Daemon can be found in the [official website](http://abyz.me.uk/rpi/pigpio/pigpiod.html).

## License

This project is licensed under the [MIT License](LICENSE) - see the [LICENSE](LICENSE) file for details. 

## Author & Maintainer
Julian Rendon

Email: julianrendon514@gmail.com