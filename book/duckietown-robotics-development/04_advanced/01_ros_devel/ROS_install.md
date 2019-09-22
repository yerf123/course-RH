
## Install ROS {#install-ROS}

This part installs ROS. You will run this twice, once on the laptop, once on the robot.

The first commands are copied from [this page][ros-ubuntu].

[ros-ubuntu]: http://wiki.ros.org/kinetic/Installation/Ubuntu

Tell Ubuntu where to find ROS:

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Tell Ubuntu that you trust the ROS people (they are nice folks):

    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Fetch the ROS repo:

    $ sudo apt update

Now install the mega-package `ros-kinetic-desktop-full`.

    $ sudo apt install ros-kinetic-desktop-full

There's more to install:

    $ sudo apt install ros-kinetic-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport,joy,image-proc,compressed-image-transport,phidgets-drivers,imu-complementary-filter,imu-filter-madgwick}

Note: Do not install packages by the name of `ros-X`, only those by
the name of `ros-kinetic-X`. The packages `ros-X` are from another version of ROS.
 

Initialize ROS:

    $ sudo rosdep init
    $ rosdep update
