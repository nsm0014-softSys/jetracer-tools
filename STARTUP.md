# NVIDIA Jetracer Startup for SOFTSYS 2022 #
The Jetracer computer is an NVIDIA Jetson Nano running a modified version of Ubuntu 20.04. ROS2 Galactic is installed to be used with this project. You will find a directory called "softsys_2022_ws" in the base directory which will have the startup packages and project you need to get started with your vehicle software development. In your vehicle shipment container (the box) you will find a joystick controller, batter, and wall adapater for the computer. Refer to the two seconds below for hardware and software introduction:

## Hardware Setup ##
### Power ###
The jetson can be powered in three main ways: AC wall adapter, Duracell Battery over micro-USB and Duracell over barrel jack. In order to change between the barrel jack and the micro-USB for power, a physical jumper connection must be made on the Jetson Nano board. The jumper will be described in class by example and below with a diagram. The J48 jumper is the one that needs to be modified for different power usage. Jumper CONNECTED is barrel jack, jumper DISCONNECTED is USB power.

![](/images/jetson_jumper.png)

The barrel jack is best for connecting to the wall AC adapter while the USB power is best for operating the vehicle while moving using the Duracell battery pack.

Once a power supply has been selected. Connect the power appropriately and the computer will immediately boot to the operating system. The Power LED will be lit green if the boot up is happening successfully. 

At this point the computer should be connected to a display through the HDMI or Displayport connections to see the operating system. The system takes a few minutes to boot so be patient.

Once the operating system comes to life, continue to the software setup section below.

### Servos ###
The servo motors are on their own power supply underneath the computer mount. The servos also have their own ON/OFF switch at the base of the vehicle. This switch should always be off unless the vehicle is planned to move immediately. The servos require an I2C PWM signal to operate and be configured after each power cycle. If there is not PWM being supplied the servos will beep until a signal is provided. The servos will also drain power from the quick discharge batteries if the switch is left on regardless of if the vehicle is moving or not. The servos are very powerful so ensure that someone is close to the ON/OFF switch while testing on the table to avoid burning any servos. They are set up to be safe in the softsys_joy software package but in case a signal is corrupted be vigilent while testing.

## Software Setup ##
If you have not read the section above about correctly powering the Jetson Nano please stop here and read that section.

Once the Jetson Nano is booted successfully, you will be greeted by Ubuntu 20.04 with an NVIDIA flavor to it. This version of Linux is built by NVIDIA to work with the specific hardware of the Jetson Nano including the CUDA cores. 

### Power Options ###
In the top right of the desktop there is label that says "MAXN" or "5W" which indicates the current Jetson power mode. MAXN will draw as much power as needed for the computer to operate at peak uncapped performance. This ends up being close to 15 watts but fluctuates. "5W" mode restricts the performance of the computer by disabling 2 of the availible 4 processing cores and capping the clock speed of the GPU cores. 5W mode will likely be needed while the computer is powered from the Duracell battery since the Duracell can only provide 10 watts max. If the computer tries to draw more it will simply shut down mid operation which is not ideal. I have added a custom power mode that hopes to meet in the middle called "10W". This 10W mode should be closer to a true maximum power of 10 watts. 10W disables 1 of the 4 CPU cores and imposes a more generous maximum clock speed across the board. From my tests, this power mode works basically all the time but your mileage may vary.

### ROS2 Workspace Setup ###
ROS2 Galactic is installed by default on the system which includes a number of custom dependencies made for the Jetson series like OpenCV and the image processing pipeline. The envirnment also should include the libi2c needed for servo control but if the workspace does not build refer to the softsys_joy readme.md for the libi2c install. 

A default workspace is provided with the ROS2 tools that are provided for this class. The directly can be found by opening a new terminal and entering the command below to change directories:

~~~
cd ~/softsys2022/softsys_2022_ws
~~~

This workspace contains source packages needed to start ROS2 development. Once your terminal is in the softsys_2022_ws, you can enter the following command to make sure the workspace builds properly without any modification needed:

~~~
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
~~~

This will build the workspace in RELEASE mode which optimizes the build for performance gains. The "--symlink-install" will ensure that your workspace uses symbolic links to things like launch files and config files. This enables you to edit your config file without recompiling the workspace for changes to take effect.

If the workspace builds successfully, congrats! You are ready to drive! To actually get your car running check below:

1. SOURCE YOUR WORKSPACE
~~~
source install/setup.bash
~~~
2. Launch the correct launch file to enable the i2c servos, joystick control and marvelmind positioning:
~~~
ros2 launch softsys_joy softsys_joy_master.launch.py
~~~
3. Power on the servos by the switch on the bottom chasis tray. Ensure they do not continuously beep!
4. Enable manual control by pressing the left shoulder button on the controller. Controls will be laid out in the softsys_joy readme.md

If the joystick controller is working properly, check out some of the data that is moving in ROS2 using the following few commands:

~~~
ros2 topic list
ros2 topic echo <name_of_topic>
~~~
~~~
rqt_graph
~~~
~~~
ros2 node list
~~~

### Autonomy Setup ###
Now that you have the base vehicle software running, you will eventually want to implement a way to control the vehicle autonomously given some criteria. The easiest way to start this will be to create a new package in the /softsys_2022_ws/src folder like the commands below:

~~~
cd ~/softsys2022/softsys_2022_ws
cd src
ros2 pkg create --node-name <node_name> <package_name>
~~~
This will create a blank package for you as you have seen in the past with an empty node class. 

You can now use this new package to start developing your own ROS2 package for controlling the vehicle. 
You will need to publish commands to /softsys/steer_cmd and /softsys/throttle_cmd to make the vehicle move according to your own choice of localization and control.