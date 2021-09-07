# Vision-Based-Control
code for "Adaptive  Vision-Based  Control  of  Redundant  Robots  with  Null-SpaceImpedance  for  Human-Robot  Collaboration"

## Prerequisites

* Ubuntu 18.04
* ROS Melodic
* Unity -to add-
* [Unity ML-Agents Toolkit](https://github.com/Unity-Technologies/ml-agents) (Recommended to read the document ahead)

For real experiments:
* [Balser camera driver](https://zh.docs.baslerweb.com/camera-installation) acA1440-220uc
* UR5
* Hololens 2


## Installation

Install the following dependences in your python3 env:

```
pip install numpy
pip install matplotlib
pip install rospkg
pip install PyYAML
pip install urdf-parser-py
pip install pykdl_utils
```

Then, clone the repo:

```
git clone
```

Build the catkin workspaces:

```
cd <YOUR_PATH>/Vision-Based-Control/ws_icra2022
catkin_make
```
