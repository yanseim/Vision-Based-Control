# Hololens2 Interface deployment

**Table of Contents**
- [Prerequisites](#prerequisties)
- [Setting up Unity Scene](#setting-up-unity-scene)
- [Import required packages](#import-required-packages)
- [Test it on your computer](#test-it-on-your-computer)
- [Deploy on HoloLens2](#deploy-on-hololens2)

## Prerequisites

Unity 2020.3.12f1c1 or later

## Setting up Unity Scene

- Download `hololens_unity_project/VBCHoloInterface.unitypackage`
- Create an empty Unity project and open it in Unity editor.
- Import the package to your project and open `Scene/HoloInterface` scene.

## Import required packages

Follow [this tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md) to import both **ROS-TCP-Connector** and **URDF-Importer** to the project. The project works on v0.5.0.

Follow [this tutorial](https://docs.microsoft.com/en-us/learn/modules/learn-mrtk-tutorials/1-3-exercise-configure-unity-for-windows-mixed-reality) to import and configure **mixed reality toolkits** to the project. Notice that the project was tested on MRTK 2.7.0.

## Test it on your computer

Select **ROSConnection** component in the hierarchy window. Then, change the IP address according to your ROS server's address.

Start the ROS server and click the "run" button, then you shall see a HUD on the upper-left corner of the window, with an increasing number indicating messages sent. If you see red arrows on the HUD, please check the IP address or refer to the log of ROS-TCP-Endpoint.

## Deploy on HoloLens2

Follow [this tutorial](https://docs.microsoft.com/en-us/learn/modules/learn-mrtk-tutorials/1-7-exercise-hand-interaction-with-objectmanipulator) to deploy the interface to your HoloLens 2 finally.

If errors occur when building the Unity project, you may want to switch on the compatibility for **UWP platforms** for all `*.dll` files in both **ROS-TCP-Connector** and **URDF-Importer** package.
