# ros_adapter - old version; skip to unity ros proxy

This repo includes functionality for interfacing with Gemini over gRPC and publish 
sensor data over ROS such that it is available to other ROS nodes.

## Getting started

It is recommended to create a `virtualenv` and then install the requirements 
running `pip install -r requirements.txt`.

The easiest way to build the container is to copy the Docker file from 
`ros_adapter/docker/Dockerfile` to the same directory as the 
`ros_adapter` directory lies, which would look something like the following:

* my_workspace
    - Dockerfile
    - ros_adapter/
    - ...

Then run the following command: `docker build -t ros_adapter .`, which will pull and build 
all you need for the container.

## NOTE: See this stackoverflow for how to set own network with static IP: https://stackoverflow.com/questions/27937185/assign-static-ip-to-docker-container

For starting and running the container, run the following command: 
`docker run -it --net=host --name ros_adapter ros_adapter`


This will start the container and attach you to an interactive shell.
Change into the `gemini_ws` directory and run: `catkin_make` followed by
`source devel/setup.bash`. The last thing needed to run the server
is to set the `ip` in the `scripts/server.py` and run roslaunch for
the `launch/launch_server.launch`.


## TODOS

    * Move most of the configuration into a script for increased automation

RUN cd /tmp && git clone -b v1.34.1 --depth 1 https://github.com/grpc/grpc && cd grpc && git submodule update --init && make grpc_php_plugin


# unity ros proxy

Clone and build with `catkin build`

Make sure you have installed grpcio, grpcio-tools, protobuf. If not:

`pip3 install grpcio`

`pip3 install grpcio-tools`

`pip3 install protobuf`



Start the server.py script or launch the _launch_ file.

### GRPC plugin install
gRPC plugins don't come with python grpcio package.


We must clone the source:
`git clone --recursive https://github.com/grpc/grpc`
Reset to v1.34.x commit:
`cd grpc && git reset --hard 3530b9c`
Compile:
`make plugins -j 12`
Now plugins are available in:
`ls bins/opt`

#### Protos in VS code:
Install vscode-proto3 plugin:
Ctrl+P and paste:
`ext install zxh404.vscode-proto3`

Edit unityrossample-ros/.vscode/settings.json:

add 
```
"protoc": {
		"path": "/usr/bin/protoc",
		"options": [
			"--plugin=protoc-gen-grpc_python=PATH_TO_grpc_python_plugin",
			"--plugin=protoc-gen-grpc_csharp=PATH_TO_grpc_csharp_plugin",
			"--proto_path=${workspaceRoot}/protos/",
			"--python_out=${workspaceRoot}/scripts/protobuf",
			"--grpc_python_out=${workspaceRoot}/scripts/protobuf",
			"--csharp_out=${workspaceRoot}/cs_scripts/protobuf",
			"--grpc_csharp_out=${workspaceRoot}/cs_scripts/protobuf"
		]
	}
```
and configure paths to python and c_sharp plugins you've built previously.

Compiling:

Ctr+Shift+P

proto3: Compile This Proto



