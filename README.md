### Project : Autonomous Landing Drone - Sejong University 2019 
License: iNCSL 
Author : Viet
Email: hoangvietdo@sju.ac.kr

# What is ROS/MAVROS ? #
http://www.ros.org/about-ros


# What is jMAVSim and Gazebo? Which one should we choose for simulation ? #

...

# How to install ROS in ubuntu 18.xx ~~~ ROS Melodic  #
For more detail and explaination please visit:
http://wiki.ros.org/melodic/Installation/Ubuntu 

# Open terminal and paste this code #
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Please read the note downbelow before use this line #
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update: 05/2019 they changed the key #
# Remove the old key #
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116

# Add the new key #
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop
# Dont use desktop-full install because we want to install Gazebo seperately #

sudo rosdep init

rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Check if environment is properly setup #
printenv | grep ROS

# Change distro to your ROS Distribution such as : indigo, kinetic, melodic #
source /opt/ros/distro/setup.bash


# Create workspace #
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/

catkin init

catkin build

source devel/setup.bash
		
# Check if you doing properly #
echo $ROS_PACKAGE_PATH 

--> output will be like this " /home/viet/catkin_ws/src:/opt/ros/melodic/share "

# NOTE:
I'm gonna cut the process here because we only have 1 more application to install which is gabezo, 
but from now on we actually dont need it yet, jMAVSim is enough for chicken final term ._. 
		
# Optional : Install jMAVSim #
git clone https://github.com/PX4/jMAVSim.git

# Note:
This code line downbelow is using for install java 8 in repository, I'm not sure it works with jMAVSim but in my case
I have to install java oracle 8 to run jMAVSim. It depends on situation please scrolldown and read my explantion about this.

sudo apt-get install ant openjdk-8-jdk openjdk-8-jre

cd jMAVSim

git submodule init

git submodule update

ant

java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -serial /dev/ttyACM0 921600 -qgc

# Switch java version to run jMAVSim if you using more than 1 version of java #
sudo update-alternatives --config java

# Setup java home and path #
Take information of java's path using the code above

export JAVA_HOME = "your_path"

export PATH=$PATH:$JAVA_HOME/bin


source ~/.bashrc

Double check: 

echo $JAVA_HOME$

java -verion
or java --version

# You will see something like this #
$ openjdk version "1.8.0_212"
$ OpenJDK Runtime Environment (build 1.8.0_212-8u212-b03-0ubuntu1.18.04.1-b03)
$ OpenJDK 64-Bit Server VM (build 25.212-b03, mixed mode)
 
# Update: 
To Solve the problem while trying to run jMAVSim in ubuntu 18.04, we need to install java 8-171 oracle jdk. java openjdk cant run jMAVSim sadly ^^! To install java 8 oracle jdk, we need to mannually install it because PPA is discontinued and oracle has changed the policy since April 2019.

# Solved : 
Because of the above reason, we need to delete jMAVSim, install java oracle version 171 and then reinstall jMAVSim.
# Please carefully check the java environment before reinstall jMAVSim #


# Gazebo installation #
Please follow step by step in this website 
http://gazebosim.org/tutorials?tut=install_ubuntu

# To use all provided utilities,  there are some packages we need to install first: #
pip install \
	pandas \
	jinja2 \
	pyserial \
	cerberus \
	pyulog \
	numpy \
	toml \
	pyquaternion

sudo apt install -y \
	ninja-build \
	exiftool \
	python-argparse \
	python-empy \
	python-toml \
	python-numpy \
	python-yaml \
	python-dev \
	python-pip \
	ninja-build \
	protobuf-compiler \
	libeigen3-dev \
	genromfs

sudo apt install ros-melodic-gazebo-ros-pkgs

# PX4 firmware and initialize a PX4 object #
cd ~/catkin_ws/src

# It could take a while #
git clone https://github.com/PX4/Firmware.git

cd Firmware

# Im not sure what this line is but it appeared in the tutorial ... :) #
git checkout v1.8.0

make posix_sitl_default gazebo

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/

install_geographiclib_datasets.sh

# Continue...




