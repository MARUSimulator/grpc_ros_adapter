# grpc_ros_adapter

## Introduction

This repo includes functionality for interfacing with Marus over gRPC and publish sensor data over ROS such that it is available to other ROS nodes.

Unity package is maintained in [marus-core](https://github.com/MARUSimulator/marus-core) repository.

Proto messages are maintained in [marus-proto](https://github.com/MARUSimulator/marus-proto) repository.

## Getting started
Recommended ROS distribution is Noetic.
 Setup:
* Clone this repository in your catkin workspace.
After cloning, run following command to pull latest proto generated source files:
`git submodule update --init`

* It is also recommended to create a `virtualenv` and then install the requirements:
`pip install -r requirements.txt`

* Clone [uuv_sensor_msgs](https://github.com/labust/uuv_sensor_msgs) in your workspace

* Build with `catkin build`

* If you encounter build errors, you can try building with python3:
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m`

* `source devel/setup.bash`

* Start grpc server:
`roslaunch grpc_ros_adapter launch_server.launch`

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