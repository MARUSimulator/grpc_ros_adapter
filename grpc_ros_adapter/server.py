#!/usr/bin/env python3

from concurrent import futures
import rospy
import grpc

from protobuf import ping_pb2_grpc
from protobuf import sensor_streaming_pb2_grpc
from protobuf import remote_control_pb2_grpc
from protobuf import tf_pb2_grpc
from protobuf import parameter_server_pb2_grpc
from protobuf import simulation_control_pb2_grpc
from protobuf import visualization_pb2_grpc
from protobuf import rf_coms_pb2_grpc

from services.ping_service import PingService
from services.sensor_streaming import SensorStreaming
from services.remote_control import RemoteControl
from services.parameter_server import ParameterServer
from services.frame_service import FrameService
from services.simulation_control import SimulationControl
from services.visualization import Visualization
from services.lora_transmission import LoraTransmission
from services.sensor_callbacks import *

from utils.ros_registry import RosRegistry

def serve(server_ip, server_port):
    """
    Add service handles to server and start server execution
    """

    options = [('grpc.max_receive_message_length', 100 * 1024 * 1024)]
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=20), options = options)


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
        "StreamLidarSensor" : [ publish_pointcloud ]
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

    rf_coms_pb2_grpc.add_LoraTransmissionServicer_to_server(
            LoraTransmission(), server
    )

    RosRegistry.remap_proto2ros('rfcommunication', 'mbzirc_msgs')
    server.add_insecure_port(server_ip + ':' + str(server_port))
    print(server_ip + ":" + str(server_port))
    server.start()
    rospy.spin()
#     server.wait_for_termination()



if __name__ == '__main__':

    rospy.init_node('syntetic_data')
    try:
        server_params = rospy.get_param('~')
        server_ip = server_params["server_ip"]
        server_port = server_params["server_port"]
    except:
        server_ip = "[::]"
        server_port = 30052

    serve(server_ip, server_port)
