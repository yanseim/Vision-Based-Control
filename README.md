# Vision-Based-Control
code for "Adaptive  Vision-Based  Control  of  Redundant  Robots  with  Null-Space Impedance  for  Human-Robot  Collaboration", ICRA 2022

## Prerequisites

* Ubuntu 18.04
* ROS Melodic
* Unity 2020.3.12f1c1 (2020.3.8f1 or later shall also work)
* [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* [Mixed Reality Toolkit](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/unity/choosing-unity-version)
  * Mixed Reality Toolkit Fondation 2.7.2
  * Mixed Reality Toolkit Standard Assets 2.7.2
  * Mixed Reality OpenXR Plugin 1.0.2
* [Hololens 2 Emulator](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/install-the-tools?tabs=unity)

For real experiments:
* [Balser camera driver](https://zh.docs.baslerweb.com/camera-installation) acA1440-220uc
* UR5
* Hololens 2

## Installation

Install the following dependences in your python3 env:

```shell
pip install numpy
pip install matplotlib
pip install rospkg
pip install PyYAML
pip install urdf-parser-py
pip install pykdl_utils
```

Then, clone the repo:

```shell
git clone
```

Build the catkin workspaces:

```shell
cd <YOUR_PATH>/Vision-Based-Control/ws_icra2022
catkin_make
```

## Packages

all the packages are in /Vision-Based-Control/ws_icra2022/src
* **aruco_ros**: for aruco. cloned from [pal-robotics/aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/melodic-devel), no modification.
* **dragandbot_common**: a driver for pylon camera, cloned from [dragandbot/dragandbot_common](https://github.com/dragandbot/dragandbot_common), no modification.
* **pylon-ros-camera**: a driver for pylon camera cloned from [basler/pylon-ros-camera](https://github.com/basler/pylon-ros-camera), where i modified ``default.yaml`` for calibration according to the README in it.
* **fmauch_universal_robot** and **Universal_Robots_ROS_Driver**: drivers for UR5, cloned from [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver), no modification. ----*submodule*
* **easy_handeye**: for eye hand calibration. i modified `ur5_basler_calibration.launch`
* **ROS-TCP-Endpoint**: for communication between Unity and ROS
* **my_pkg**: main control code
* **tcst_pkg**: control code we refered to. from [Musyue/tcst_pkg](https://github.com/Musyue/tcst_pkg). ----*submodule*

## Usage

```
cd ws_icra2022
source devel/setup.bash
roslaunch my_pkg ur5_bringup.launch # for UR5
roslaunch ros_tcp_endpoint endpoint.launch # for hololens 2
roslaunch my_pkg ibvs_run_eye2hand.launch # for control
```

