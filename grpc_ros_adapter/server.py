#!/usr/bin/env python3

from concurrent import futures
import grpc_ros_adapter.utils.ros_handle as rh
import grpc

from grpc_ros_adapter.protobuf import ping_pb2_grpc
from grpc_ros_adapter.protobuf import sensor_streaming_pb2_grpc
from grpc_ros_adapter.protobuf import remote_control_pb2_grpc
from grpc_ros_adapter.protobuf import tf_pb2_grpc
from grpc_ros_adapter.protobuf import parameter_server_pb2_grpc
from grpc_ros_adapter.protobuf import simulation_control_pb2_grpc
from grpc_ros_adapter.protobuf import visualization_pb2_grpc

from grpc_ros_adapter.services.ping_service import PingService
from grpc_ros_adapter.services.sensor_streaming import SensorStreaming
from grpc_ros_adapter.services.remote_control import RemoteControl
from grpc_ros_adapter.services.parameter_server import ParameterServer
from grpc_ros_adapter.services.frame_service import FrameService
from grpc_ros_adapter.services.simulation_control import SimulationControl
from grpc_ros_adapter.services.visualization import Visualization
from grpc_ros_adapter.services.sensor_callbacks import *

def serve(server_ip, server_port):
    """
    Add service handles to server and start server execution
    """
    MAX_MESSAGE_LENGTH = 100 * 1024 * 1024
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=20),
    options=[
        ('grpc.max_send_message_length', MAX_MESSAGE_LENGTH),
        ('grpc.max_receive_message_length', MAX_MESSAGE_LENGTH),
    ])


    ping_pb2_grpc.add_PingServicer_to_server(
            PingService(),
            server)

    sensor_streaming_callbacks =  {
        "StreamCameraSensor": [ publish_image ],
        "StreamImuSensor": [ publish_imu ],
        "StreamPoseSensor": [ publish_pose ],
        "StreamDepthSensor": [ publish_depth ],
        "StreamDvlSensor": [ publish_dvl ],
        "StreamSonarSensor": [ publish_sonar ],
        "StreamSonarFixSensor": [ publish_sonar_fix ],
        "StreamAisSensor" : [ publish_ais ],
        "StreamGnssSensor" : [ publish_gnss ],
        "StreamLidarSensor" : [publish_pointcloud],
        "StreamPointCloud2" : [publish_pointcloud2]
    }

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            SensorStreaming(sensor_streaming_callbacks),
            server)

    remote_control_pb2_grpc.add_RemoteControlServicer_to_server(
            RemoteControl(), server
    )

    tf_pb2_grpc.add_TfServicer_to_server(
            FrameService(), server
    )

    parameter_server_pb2_grpc.add_ParameterServerServicer_to_server(
            ParameterServer(), server
    )

    simulation_control_pb2_grpc.add_SimulationControlServicer_to_server(
            SimulationControl(), server
    )

    visualization_pb2_grpc.add_VisualizationServicer_to_server(
            Visualization(), server
    )

    server.add_insecure_port(server_ip + ':' + str(server_port))
    rh.loginfo(server_ip + ":" + str(server_port))
    server.start()
    rh.spin()
#     server.wait_for_termination()


def main():
    rh.init('syntetic_data')
    server_ip = rh.get_param("~server_ip") or "localhost"
    server_port = rh.get_param("~server_port") or 30052

    serve(server_ip, server_port)

if __name__ == '__main__':
    main()
