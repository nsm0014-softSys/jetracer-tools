Packages available to use out of the box for students:

1. i2c_pwm_ros2: I2C PWM low level controller to translate and calibrate motion of continuous servo motors. Steering and acceleration. This package will be needed to calibrate the steering of your servos if desired as well as adjust the limits of acceleration.
2. marvelmind_ros2: The Marvelmind package provides low level tools which read device messages and translates them into usable ROS2 topics. The package is described in detail below.
3. softsys_joy: The Softsys Joy (joystick) package will be used as the buffer layer between the low level drivers mentioned above and the high level control from the user. The Joy package exposes two topics for actually controlling the vehicle through steering commands and throttle commands. The package also integrates joystick control of the vehicle between the low level and high level control to provide easy testing of prototype code. More detail below.
4. image_pub: The image publisher package is responsible for taking raw camera signals and converting it to a user friendly ROS2 image format which can be used by the user for image processing. This package can also be used to calibrate the camera parameters if desired. The parameters are all set for this project and should not be changed to avoid risking destroying servos.

### I2C PWM Package ###
The I2C PWM package comes in two parts. The main package where classes and functionality is written and a message package for custom message support. The package works on the basis of an i2c library publicly availible for microcontrollers called libi2c. The package uses the libi2c function calls to manipulate the physical servo behavior. This package has a number of config options that can be found in the config.yaml file. The following parameters may be inspected:

1. logger_level: Describes the logger level output of the package, INFO by default will let you know that the package initializes and starts up correctly. DEBUG will include extra debug output which is likely not needed. WARN and ERROR will have no info messages unless critical errors or warnings are thrown.
2. i2c_device_address: The i2c address describes where the program should look for an i2c specific device to be connected. In most cases the default should not changed since i2c devices should always fall under the /dev/i2c-# naming convention.
3. controller_io_device: the i2c device on the given i2c board. In our case the first i2c module on the borad is 1.
4. controller_io_handle: The i2c board that is connected to the computer. Should always be 0.
5. pwm_frequency: The pulse width modulation frequency can be changed to support specific servos or tweaked slightly for better responsiveness of the current servo. 50hz is the default for our servos.
6. active_board: Directs the program to which board is currently being looked at out of possible boards. There can be up to 16 boards but our active board is always 1 since we only have one i2c board connected.
7. drive_config: The i2c package supports specific drive modes since it was written for 4 wheel ground vehicles but we do not implement drive modes so none of the parameters are important.

This package will not need to be used by itself and relies on the softsys_joy package to function so it is included in the softsys_joy launch file.

### Marvelmind ROS2 ###
The Marvelmind package is a ROS2 implementation of a C library provided by the Marvelmind company. This node interacts with the Hedgehog device to provide ROS2 topics providing information about beacon position and base station locations. The package is already set up for use with the Jetracer projcet and should likely not need to be changed. Most of the settings are changed through the desktopi application of the Marvelmind system meaning the ROS2 parameters will not address all settings. There are three main settings to be looked at:

1. anything /* or "topic": Most of the topic names are in the config and can be changed for convinience but otherwise not needed.
2. publish_rate_in_hz: The publish rate can be adjusted for performance reasons or to meet performance requirements better. 50hz seems appropriate as the device can only update at around 100hz maximum.
3. tty_baudrate: This is the serial port baud rate which should not be changed.

This package can be launched with the following:

~~~
source install/setup.bash
ros2 launch marvelmind_ros2 marvelmind_ros2.launch.py
~~~

### Image Publisher ###
The image_pub package provides a way to get live video from the front facing CSI camera on the Jetracer. This package works on the basis of a GStreamer Image Pipeline in coordination with OpenCV image format conversion to provide a color image stream over ROS2. The package config file has numerous camera parameters that can be tweaked but the base implementaiton does not use them to make any corrections. The base implementaiton only gives the raw camera output with no calibration or tuning applied. You may look at the example C++ projects to incorporate image calibration and correction. The package has a few important config options described below:

Inside imx21983_raw_image_pub.yaml...
1. framerate: This value sets the desired framerate of the camera. Cameras only support a few frame rate configurations so best to leave this at 60 since the sensor does not support slower.
2. sensor_id: The sensor_id changes the camera sensor mode. The CSI camera has 5 modes and mode 4 is the best suited for our application since it is the lowest resolution and framerate therefore giving the best performance.
3. namespace: The namespace to be applied to the output image topic, can be changed if needed.
4. frame_id: Image can be taked with a frame id to describe the location of the camera in space but we do not use this.
5. pipe: The pipe command strings describe how GStreamer will construct a pipeline, do not change.

The node will publish two topics. The useful topic will be /softsys/image_raw which will contain the actual image stream from the camera.

The image_pub can be launched by the following:
~~~
ros2 launch image_pub raw_image_pub.launch.py
~~~

### Softsys Joy ###
The Softsys Joy package will be the main way students interact with the vehicle through ROS2. The softsys joy package is designed to be the middle man between the low level control of servos and the high level autonomy control from steering and localization. The package features joystick control built in and the ability to switch between autonomous and manual mode with one button press (the left shoulder button). The package also features some safety features to promote healthy operation of our vehicles. The system will not run if the joystick device stops communicating with your vehicle. For example if your joystick controller batteries die, the vehicle will come to a stop. A better example is if you are out of reach of your vehicle. Another important safety feature is separated steering and throttle autonomy. With the joystick you can enable autonomous throttle OR autonomous steering so that you can test one without the other. This is handy to testing your steering implementation with a manual throttle control. This package is also accompnaied by a messages package with the steering and throttle message types. Reference these messages when attempting to steer or accelerate the vehicle from your own lateral or longitudinal controllers. When it comes time, you can simply add the package to your own CMakeList.txt to include it:

~~~
find_package(softsys_msgs REQUIRED)
~~~

The package will expose two main topics for vehicle control:

~~~
/softsys/steer_cmd
/softsys/throttle_cmd
~~~

1. /softsys/steer_cmd will allow you to steer the vehicle. This topic accepts a steering percentage. The percentage ranges from -1 to 1 where a command of 0 means steering straight ahead.
2. /softsys/throttle_cmd will allow you to accelerate the vehicle. This topic accepts a throttle percentage. The percentage ranges from -1 to 1. A throttle command of -1 will be full reverse and a throttle command of 1 will be full forward. A command of 0 will allow the vehicle to roll freely.

The package has a number of parameters that will be important to look through as you tune your vehicle for competition. The most important will have to do with the servo values listed at the end. 

1. servo_update_rate_seconds: Describes the update rate of the servo control callback. This is up do the user but the 10 millisecond default is more than appropriate for class use.
2. joy_timeout: The timeout in seconds that is required before a safe stop occurs from controller disconnect. This should not be greater than 1.0 to avoid unneeded collisions.
3. auto_timeout: The timeout in seconds that the vehicle will swap to manual mode if no autonmous input is received. This prevents unintended run aways and side effects.
4. autonomous_switching_delay: The delay between being able to switch modes of control from autonomous to manual and back. This is basically button debounce to make sure your button presses do what you expect.
5. drive_servo_num and steer_servo_num: Servo number connected to i2c board which specify the type of control to send to each. These should NOT be changed.
6. min_steer_value: PWM value describing lowest signal for the steering servo.
7. max_steer_value: PWM value describing highest PWM signal for the steering servo.
8. min_throttle_value: PWM value for lowest throttle command. PWM is backwards so the lower this number the FASTER your servo will spin.
9. max_throttle_value: PWM value for highest throttle command. PWM is backwards so the lower this number the SLOWER (or backwards) your servo will spin.

The last four parameters can be tuned to your vehicle and likely to fix steering offset or fix deadbanding in your throttle servo. There is a specific value that will engage the servo to the point where it spins so setting this value as your starting point may be helpful.

The softsys joy package contains two main launch files: launching just softsys_joy and launching all nodes needed for full operation. The second option will be preffered so that the i2c, marvelmind, and lowlevel joystick packages are all launched together.

Softsys Joy only:
~~~
ros2 launch softsys_joy softsys_joy_base.launch.py
~~~

Master Launch:
~~~
ros2 launch softsys_joy softsys_joy_master.launch.py
~~~

