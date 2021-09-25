# Vision-Based-Control
code for "Adaptive  Vision-Based  Control  of  Redundant  Robots  with  Null-SpaceImpedance  for  Human-Robot  Collaboration"

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
* **aruco_ros**: for aruco
* **dragandbot_common** and **pylon-ros-camera**: drivers for pylon camera
* **fmauch_universal_robot** and **Universal_Robots_ROS_Driver**: drivers for UR5
* **easy_handeye**: for eye hand calibration.`roslaunch easy_handeye ur5_basler_calibration.launch`
* **ROS-TCP-Endpoint**: for communication between Unity and ROS
* **my_pkg**: main control code
* **tcst_pkg**: control code we refered to. from [Musyue/tcst_pkg](https://github.com/Musyue/tcst_pkg)
