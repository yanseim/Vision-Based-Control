# Vision-Based-Control
code for "Adaptive  Vision-Based  Control  of  Redundant  Robots  with  Null-SpaceImpedance  for  Human-Robot  Collaboration"

## Prerequisites

* Ubuntu 18.04
* ROS Melodic
* Unity 2020.3.17f1 LTS ()
* [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* [Mixed Reality Toolkit](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/unity/choosing-unity-version)
  * Mixed Reality Toolkit Fondation 2.7.2
  * Mixed Reality Toolkit Standard Assets 2.7.2
  * Mixed Reality OpenXR Plugin 1.0.0
* [Unity ML-Agents Toolkit](https://github.com/Unity-Technologies/ml-agents) (Recommended to read the document ahead)
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
