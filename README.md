# unityrossample-ros
Sample project for unity-ros integration

## Installation

Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Inside, pull [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) and unityrossample:
```
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
git clone https://github.com/labust/unityrossample-ros.git
```

## Start

After building and sourcing whole workspace, run:

```
roslaunch unitysample-ros server.launch
```

