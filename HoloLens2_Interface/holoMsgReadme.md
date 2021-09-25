# CustomMsgTest

## ROS

![](https://github.com/Unity-Technologies/Unity-Robotics-Hub/raw/main/tutorials/ros_unity_integration/images/unity_ros.png)

### Manually


```bash
hostname -I #get IP address
rosparam set ROS_IP <your IP address>
rosparam set ROS_TCP_PORT <port number> #Optional. set the port, server will run on port 10000 by default.
```

```bash
 rosrun ros_tcp_endpoint default_server_endpoint.py #run Server Endpoint
```

### Launch file

```bash
echo "ROS_IP: $(hostname -i)" > src/ros-tcp-endpoint/config/params.yaml #set the IP address
echo "ROS_IP: 192.168.8." > src/ros-tcp-endpoint/config/params.yaml #set the IP address
roslaunch ros_tcp_endpoint endpoint.launch #run Server Endpoint
```

