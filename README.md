## Intelligent Navigation and Control Systems Laboratory ##

##### How to install ROS/MAVROS/GAZEBO in Ubuntu 18.04/16.04

License: [iNCSL](https://sites.google.com/view/incsl)

Author : Viet , San Hee Kang

Email: hoangvietdo@sju.ac.kr , rkdovo08@naver.com

#### What is ROS/MAVROS ? 
[ROS/MAVOS](http://www.ros.org/about-ros)

#### What is jMAVSim and Gazebo? Which one should we choose for simulation ? 
jMAVSim is a simple version of drone simulation. It's easy to install and run without any complicated step.
However, jMAVSim has a limitation in application and modifying. jMAVsim only suitable for implementing
control theory.

Gazebo, in another hand, is a powerful simation, it support every kind of application (ground,aerial,..).
Likewise, Gazebo also support vision-based application (opencv, SLAM, AI,..). Eventhough, along with its pros, gazebo requires a lot of knowledge in programming to code and operate.
[GAZEBO](http://gazebosim.org/)

In this Project, we will use Gazebo.

#### Requirements:
1. ROS
2. MAVROS 
3. Gazebo ( Project version : 9)
4. Opencv 3+ ( Project version: 3.2.0 )
5. Java 8+ ( Project version: 11 )
6. PX4 Firmware v1.8.2

#### Note:
- Please carefully read and follow this instruction.

- jMAVSim installation section is not important, if you have to use jMAVSim you can refer to this instruction. Because we will not use jMAVSim as I explained above section, you dont need to install jMAVSim

#### 1. How to install ROS in ubuntu 18.xx ~~~ ROS Melodic  
- Reference: [Ubuntu18.04](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Check for update/upgrade:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get autoremove
```
- Open terminal and paste this code:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```
- Check if environment is properly setup:
```
printenv | grep ROS
```
- Install and config cv_brigde for ROS and OpenCV

Still Working on it ...

#### 2.Install MAVROS 
- Reference : [MAVROS](https://dev.px4.io/en/ros/mavros_installation.html)
```
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
sudo apt-get install python-catkin-tools -y
```
- Create workspace 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
cd ~/catkin_ws/src
catkin_init_workspace
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
cd ..
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
- Check if you doing properly 
```
echo $ROS_PACKAGE_PATH 
```
- Output will be like this:
```
/opt/ros/melodic/share
```
#### 3. Gazebo installation 
- Reference: [GAZEBO Installation](http://gazebosim.org/tutorials?tut=install_ubuntu)

- To use all provided utilities,  there are some packages we need to install 
first:
```
sudo apt install python-pip
pip install pandas jinja2 pyserial cerberus pyulog numpy toml pyquaternion
```
- If there is any error, please substitute "pip -install -y" with:
```
pip --user install
```
- And:
```
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
```
- Check installation:
```
gazebo
```
- If you have an error like this: 
```
[Err] [REST.cc:205] Error in REST request

libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
```
- Fix it with: 
```
sudo nano ~/.ignition/fuel/config.yaml
```
- and change: 
```
From : url: https://api.ignitionfuel.org

to   : url: https://api.ignitionrobotics.org
```
- Press Ctrl + O, Enter, Ctrl X and then run gazebo again. Error should be fixed.


#### 4. Install PX4 Library - Software in the loop simulation (SITL) 
```
cd ~/catkin_ws/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.8.2
git submodule update --init --recursive
sudo apt-get install python-jinja2 -y
make posix_sitl_default gazebo
cd ~/catkin_ws
catkin build
```
- Opend ~/.bashrc and add these lines at the end of the script:
```
sudo nano ~/.bashrc
source ~/catkin_ws/src/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/Firmware ~/catkin_ws/src/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Firmware/Tools/sitl_gazebo
```
- Press Ctrl+O, Enter, Ctrl+X
```
source ~/.bashrc
```
#### 5. How to install Opencv in ubuntu 18.04 and make it usable to ROS 
- Reference: [Opencv](https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/)

- Install OpenCV dependencies on Ubuntu 18.04:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev clang-3.9 lldb-3.9 -y
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev -y
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev -y
sudo apt-get install libatlas-base-dev gfortran -y
sudo add-apt-repository 'deb http://security.ubuntu.com/ubuntu xenial-security main'
sudo apt update
sudo apt install libjasper1 libjasper-dev
```
#### ( Optional ) : Install and use jMAVSim as simulation 
```
git clone https://github.com/PX4/jMAVSim.git
```
- Note:
This code line downbelow is using for install java 8 in repository, I'm not sure it works with jMAVSim but in my case.
I have to install java oracle 8 to run jMAVSim. It depends on situation please scrolldown and read my explantion about this.
```
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre
cd jMAVSim
git submodule init
git submodule update
ant
java -Djava.ext.dirs= -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -serial /dev/ttyACM0 921600 -qgc
```
- Switch java version to run jMAVSim if you using more than 1 version of java:
```
sudo update-alternatives --config java
```
- Setup java home and path: Take information of java's path using the code above
```
export JAVA_HOME = "your_path"
export PATH=$PATH:$JAVA_HOME/bin
source ~/.bashrc
```
- Double check: 
```
echo $JAVA_HOME$
java -verion
or java --version
```
- You will see something like this: 
```
$ openjdk version "1.8.0_212"
$ OpenJDK Runtime Environment (build 1.8.0_212-8u212-b03-0ubuntu1.18.04.1-b03)
$ OpenJDK 64-Bit Server VM (build 25.212-b03, mixed mode)
 ```
- Update: 
To Solve the problem while trying to run jMAVSim in ubuntu 18.04, we need to install java 8-171 oracle jdk. java openjdk cant run jMAVSim sadly ^^! To install java 8 oracle jdk, we need to mannually install it because PPA is discontinued and oracle has changed the policy since April 2019.

- Solved : 
Because of the above reason, we need to delete jMAVSim, install java oracle version 171 and then reinstall jMAVSim.
Please carefully check the java environment before reinstall jMAVSim.


#### How to start with your Project ! 
- Reference : [ROS](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- Above, we created a ~/catkin_ws/ folder which can be use as a project folder but we also can create another one for specific purposes:
```
mkdir -p ~/projectName_ws/src 
```
- Note: Here, ws stands for workspace, you can arbitrarily change to your favorite folder name and then:
```
cd ~/projectName_ws/src
catkin_init_workspace
cd ..
catkin init
catkin build
sudo nano ~/.bashrc
```
- Add this line at the end of the bash script:
```
source ~/projectName_ws/devel/setup.bash
```
- Again, save script and resource it:
```
source ~/.bashrc
```
- Create ROS folder 
```
cd ~/projectName_ws/src
catkin_create_pkg folderName std_msgs rospy roscpp
cd ..
catkin build
```
- Till now, We did successfully create a project and setup environment, workspace. Next step will be coding. There are 3 kind of file that required to make:
```
1. CMAkeList
2. .Launch
3. .cpp
```
- Launch MAVROS and Gazebo simulation at the same time:
```
roslaunch px4 mavros_posix_sitl.launch
```
- Check if everything is setup correctly: Open a new terminal window while other terminal running above code line:
```
rostopic echo /mavros/state
```
- Check whether "connected status" returns TRUE.
```
cp -r ~.../launch/* ~/catkin_ws/src/Firmware/launch/
cp -r ~.../models/* ~/catkin_ws/src/Firmware/Tools/sitl_gazebo/models/
cp -r ~.../posix-config/* ~/catkin_ws/src/Firmware/posix-configs/SITL/init/ekf2/
cp -r ~.../world/* ~/catkin_ws/src/Firmware/Tools/sitl_gazebo/worlds/
```
- To run the simulation:
```
roslaunch px4 landing.launch
```
- Open another 2 terminal and cd to the Folder that contain the python code:
```
cd ...
python px4_mavros_run.py
ipython
from commander import Commander
c = Commander()
c.move(1,2,3)
```
- To Trigger Camera:
```
rosrun rqt_image_view rqt_image_view
```
