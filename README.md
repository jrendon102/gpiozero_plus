# yb_expansion_board

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A Python Library for controlling the Yahboom 4WD expansion board.

## Dependencies

This library relies on the following dependency:

- **gpiozero**: A Python library for controlling GPIO pins on Raspberry Pi and other supported platforms. It is used to interface with the hardware components on the Yahboom expansion boards.

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

## Uninstall
You can uninstall the **yb_expansion_board** using the following command:

   ```bash
   pip uninstall yb_expansion_board
   ```

## Example
Here's an example of how to use the `yb_expansion_board` library in your Python project:

```python
import time
from yb_expansion_board import YBFan 

# Initialize and use the various components from the library
fan = YBFan(fan_pin=17)

# Turn fan on for 5 seconds
printf("Turning fan on for 5 seconds")
fan.control_fan(fan_enable=True)
time.sleep(5)

# Turn fan on off 5 seconds
printf("Turning fan off for 5 seconds")
fan.control_fan(fan_enable=False)
time.sleep(5)

# Don't forget to disconnect when you're done!
fan.disconnect()
```

## More Information
For more information about the Yahboom 4WD expansion board, including schematics, user guides and GPIO pin numbering, please visit the [officiall website](http://www.yahboom.net/study/4wd-ban).

## License

This project is licensed under the [MIT License](LICENSE) - see the [LICENSE](LICENSE) file for details. 

## Author & Maintainer
Julian Rendon

Email: julianrendon514@gmail.com