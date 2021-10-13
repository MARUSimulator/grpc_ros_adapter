#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from concurrent import futures
import grpc

from protobuf import ping_pb2_grpc
from protobuf import sensor_streaming_pb2_grpc
from protobuf import commander_service_pb2_grpc
from protobuf import remote_control_pb2_grpc
from protobuf import tf_pb2_grpc
from protobuf import parameter_server_pb2_grpc
from protobuf import simulation_control_pb2_grpc
from protobuf import acoustic_transmission_pb2_grpc

from services.ping_service import PingService
from services.sensor_streaming import SensorStreaming
from services.remote_control import RemoteControl
from services.parameter_server import ParameterServer
from services.frame_service import FrameService
from services.simulation_control import SimulationControl
from services.service_caller import ServiceCaller
from services.acoustic_transmission import AcousticTransmission

from services.sensor_callbacks import *
import utils.ros_handle as rh 

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
        "StreamSonarSensor": [ publish_sonar ],
        "StreamSonarFixSensor": [ publish_sonar_fix ],
        "StreamAisSensor" : [ publish_ais ],
        "StreamGnssSensor" : [ publish_gnss ],
        "StreamLidarSensor" : [ publish_lidar ]
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
    commander_service_pb2_grpc.add_CommanderServicer_to_server(
            ServiceCaller(),
            server)
    

    acoustic_transmission_pb2_grpc.add_AcousticTransmissionServicer_to_server(
            AcousticTransmission(), server
    )

    server.add_insecure_port(server_ip + ':' + str(server_port))
    print(server_ip + ":" + str(server_port))
    server.start()
    rh.spin()
#     server.wait_for_termination()


def main(args=None):

    _default_parameters = {
        "LocalOriginLat" : float(45),
        "LocalOriginLon" : float(12)
    }

    rh.init("ros_adapter", "ros_adapter", args)
    server_ip = rh.get_param("server_ip") or "0.0.0.0"
    server_port = rh.get_param("port") or 30053

    for k, v in _default_parameters.items():
        rh.set_param(k, v)

    serve(server_ip, server_port)


if __name__ == "__main__":
    main()




