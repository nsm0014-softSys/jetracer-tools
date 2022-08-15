# Student Repo for Softsys Tools #

For a new install, please also get the i2c lib package before building:

~~~
 sudo apt-get install libi2c-dev
~~~

To get rosdep dependencies
~~~
rosdep install -i --from-path src --rosdistro galactic -y
~~~


If rosdep does not get everything for camera dependencies:
~~~
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
~~~


~~~
sudo apt install ros-galactic-image-pipeline
~~~
