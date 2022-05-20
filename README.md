# grpc_ros_adapter

## Introduction

NOTE: ROS2 support is still experimental. You will encounter problems and bugs.

This repo includes functionality for interfacing with Marus over gRPC and publish sensor data over ROS2 galactic such that it is available to other ROS2 nodes.

Unity package is maintained in [marus-core](https://github.com/MARUSimulator/marus-core) repository.

Proto messages are maintained in [marus-proto](https://github.com/MARUSimulator/marus-proto) repository.

## Getting started
Recommended ROS2 distribution is Galactic.
 Setup:
* Clone this repository in your catkin workspace.
After cloning, run following command to pull latest proto generated source files:
`git submodule update --init`

* It is also recommended to create a `virtualenv` and then install the requirements:
`pip install -r requirements.txt`

* Clone [uuv_sensor_msgs](https://github.com/labust/uuv_sensor_msgs) in your workspace

* Build with `colcon build`

* `source install/setup.bash`

* Start grpc server:
`ros2 launch grpc_ros_adapter ros2_server_launch.py`

## Usage and documentation

For usage information and examples visit our [marus-example](https://github.com/MARUSimulator/marus-example) project repository.

For other information and documentation visit our [documentation homepage](https://marusimulator.github.io).

## Credits & Acknowledgements


* [gRPC](https://github.com/grpc/grpc)
* [protobuf](https://github.com/protocolbuffers/protobuf)
* [Gemini Unity simulator](https://github.com/Gemini-team/Gemini)


## Contact
Please feel free to provide feedback or ask questions by creating a Github issue. For other inquiries, please email us at labust@fer.hr or visit our web pages below:
* [Laboratory for Underwater Systems and Technologies - LABUST](https://labust.fer.hr/)

* [University of Zagreb, Faculty of Electrical Engineering and Computing](https://www.fer.unizg.hr/en)

## License
This project is released under the Apache 2.0 License. Please review the [License](https://github.com/MARUSimulator/grpc_ros_adapter/blob/dev/LICENSE) file for more details.