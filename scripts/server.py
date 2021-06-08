#!/usr/bin/env python3

from concurrent import futures
import rospy
import grpc

from protobuf import ping_pb2_grpc
from protobuf import navigation_pb2_grpc
from protobuf import sensor_streaming_pb2_grpc
from protobuf import remote_control_pb2_grpc

from services.ping_service import PingService
from services.sensor_streaming import SensorStreaming
from services.remote_control import RemoteControl
from services.navigation import Navigation

from services.sensor_callbacks import *

def serve(server_ip, server_port):
    """
    Add service handles to server and start server execution
    """
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=20))


    ping_pb2_grpc.add_PingServicer_to_server(
            PingService(),
            server)

    sensor_streaming_callbacks =  {
        "StreamCameraSensor": [ publish_image ],
        "StreamImuSensor": [ publish_imu ],
        "StreamPoseSensor": [ publish_pose ],
        "StreamDepthSensor": [ publish_depth ],
        "StreamDvlSensor": [ publish_dvl ],
        "StreamSonarSensor": [ publish_sonar ]
    }

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            SensorStreaming(sensor_streaming_callbacks),
            server)

    navigation_pb2_grpc.add_NavigationServicer_to_server(
            Navigation(None, None, None),
            server)

    remote_control_pb2_grpc.add_RemoteControlServicer_to_server(
            RemoteControl(), server
    )



    server.add_insecure_port(server_ip + ':' + str(server_port))
    print(server_ip + ":" + str(server_port))
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':

    rospy.init_node('syntetic_data')
    server_params = rospy.get_param('~')
    server_ip = server_params["server_ip"]
    server_port = server_params["server_port"]

    serve(server_ip, server_port)
