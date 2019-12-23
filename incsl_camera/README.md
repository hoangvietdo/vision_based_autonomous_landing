Package Requirements:
- libuvc_camera
  sudo apt-get install ros-melodic-libuvc-camera

- To find the idVendor and idProduct:
  lsusb

- To config:
  - Create file name "99-uvc.rules" in "/etc/udev/rules.d/"
  - Add this line into the created file with corresponding idVendor/idProduct:
    SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", MODE="0666"
  - In terminal:
    sudo udevadm control --reload-rules
  - Disconnect camera and replugin

- To find image parameter for launch file:
  v4l2-ctl --list-formats-ext -d /dev/video0

- To build: Go to the ~/catkin_ws/src:
  catkin build incsl_camera

- To run:
  roslaunch incsl_camera incsl_camera.launch

- Another terminal:
  rosrun rqt_image_view rqt_image_view

- To test marker detection, Open another terminal in Python folder:
  python autonomous_landing_camera.py
