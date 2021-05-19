#!/usr/bin/env python3

from concurrent import futures

import rospy
import roslib
import tf2_ros
import tf.transformations as tftrans
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from ros_adapter.msg import RadarSpoke
from rosgraph_msgs.msg import Clock
import geometry_msgs.msg as geomsgs

import grpc

from protobuf import navigation_pb2_grpc
from protobuf import sensor_streaming_pb2_grpc
from protobuf import remote_control_pb2_grpc

from services.sensor_streaming import SensorStreaming
from services.remote_control import RemoteControl
from services.navigation import Navigation



def serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          pose_pub, twist_pub, tf_pub):

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            SensorStreaming(camera_pubs, lidar_pub, radar_pub, clock_pub),
            server)

    navigation_pb2_grpc.add_NavigationServicer_to_server(
            Navigation(pose_pub, twist_pub, tf_pub),
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

    cam_ids = ["F", "FL", "FR", "RL", "RR"]
    camera_pubs = dict()
    for cam_id in cam_ids:
        camera_pubs[cam_id] = rospy.Publisher('EO/' + cam_id + '/image_raw',
                                              Image, queue_size=10)

    lidar_pub = rospy.Publisher('lidar/driver/velodyne_points',
                                PointCloud2,
                                queue_size=10)

    # TODO: Change the message type to be published
    radar_pub = rospy.Publisher('radar/driver/spokes',
                                RadarSpoke,
                                queue_size=10)

    clock_pub = rospy.Publisher('clock', Clock, queue_size=10)

    pose_pub = rospy.Publisher('milliampere/pose', geomsgs.PoseStamped, queue_size=10)

    twist_pub = rospy.Publisher('milliampere/twist', geomsgs.TwistStamped, queue_size=10)

    tf_pub = tf2_ros.TransformBroadcaster()

    serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          pose_pub, twist_pub, tf_pub)
