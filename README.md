# ros_adapter

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

