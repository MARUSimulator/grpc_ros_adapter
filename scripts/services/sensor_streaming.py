
import numpy as np
import grpc
import cv2

import rospy
from rospy import Publisher
from cv_bridge import CvBridge, CvBridgeError
import std_msgs.msg
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from ros_adapter.msg import RadarSpoke
from rosgraph_msgs.msg import Clock

from protobuf import sensor_streaming_pb2
from protobuf import sensor_streaming_pb2_grpc


_PUBLISHER_TYPES = {
    "camera": Image,
    "depth": Float32,
    "pose": Pose,
    "imu": Vector3
}

class SensorStreaming(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self, camera_topic="camera", lidar_topic="lidar", radar_topic="radar", depth_topic="depth", pose_topic="pose"):
        print("creating")
        self.bridge = CvBridge()
        self.publishers = {}

    def _get_publisher(self, pub_type, sensor_id) -> Publisher:
        # TODO: better check and logging
        if pub_type not in _PUBLISHER_TYPES:
            return None

        pub_type_dict = self.publishers.get(pub_type, {})
        if not pub_type_dict:
            self.publishers[pub_type] = pub_type_dict
        
        publisher = pub_type_dict.get(sensor_id, None)
        if not publisher:
            publisher = Publisher(f"{pub_type}/{sensor_id}", _PUBLISHER_TYPES[pub_type], queue_size=10)
            pub_type_dict[sensor_id] = publisher
        return publisher


    def StreamCameraSensor(self, request_iterator, context):
        """
        Takes in a gRPC SensorStreamingRequest containing
        all the data needed to create and publish a sensor_msgs/Image
        ROS message.
        """
        for request in request_iterator:
            img_string = request.data

            cv_image = np.fromstring(img_string, np.uint8)

            # NOTE, the height is specifiec as a parameter before the width
            cv_image = cv_image.reshape(request.height, request.width, 3)
            cv_image = cv2.flip(cv_image, 0)

            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            msg = Image()
            header = std_msgs.msg.Header()
            try:
                # RGB
                # msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

                # BGR
                msg = self.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

                header.stamp = rospy.Time.from_sec(request.timeStamp)
                msg.header = header
            except CvBridgeError as e:
                print(e)

            pub = self._get_publisher("camera", request.sensorId)
            pub.publish(msg)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamLidarSensor(self, request, context):
        """
        Takes in a gRPC LidarStreamingRequest containing
        all the data needed to create and publish a PointCloud2
        ROS message.
        """

        pointcloud_msg = PointCloud2()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.from_sec(request.timeInSeconds)

        header.frame_id = "velodyne"
        pointcloud_msg.header = header

        pointcloud_msg.height = request.height
        pointcloud_msg.width = request.width

        fields = request.fields

        # Set PointCloud[] fields in pointcloud_msg
        for i in range(len(fields)):
            pointcloud_msg.fields.append(PointField())
            pointcloud_msg.fields[i].name = fields[i].name
            pointcloud_msg.fields[i].offset = fields[i].offset
            pointcloud_msg.fields[i].datatype = fields[i].datatype
            pointcloud_msg.fields[i].count = fields[i].count

        pointcloud_msg.is_bigendian = request.isBigEndian
        pointcloud_msg.point_step = request.point_step
        pointcloud_msg.row_step = request.row_step

        pointcloud_msg.data = request.data

        pointcloud_msg.is_dense = request.is_dense

        self.lidar_pub.publish(pointcloud_msg)

        # TODO: This does not belong in this RPC implementation, should be
        # moved to own or something like that.
        sim_clock = Clock()
        sim_clock.clock = rospy.Time.from_sec(request.timeInSeconds)
        self.clock_pub.publish(sim_clock)

        return sensor_streaming_pb2.LidarStreamingResponse(success=True)

    def StreamRadarSensor(self, request, context):
        """
        Takes in a gRPC RadarStreamingRequest containing
        all the data needed to create and publish a RadarSpoke
        ROS message.
        """
        
        number_of_spokes = request.numSpokes

        for i in range(number_of_spokes):

            radar_spoke_msg = RadarSpoke()

            # Header
            header = std_msgs.msg.Header()
            header.frame_id = "milliampere_radar"
            header.stamp = rospy.Time.from_sec(request.timeInSeconds[i])
            # radar_spoke_msg.azimuth = request.azimuth[i]
            # radar_spoke_msg.intensity = request.radarSpokes[i * request.numSamples : i * request.numSamples + request.numSamples]

            # radar_spoke_msg.range_start = request.rangeStart
            # radar_spoke_msg.range_increment = request.rangeIncrement
            # radar_spoke_msg.min_intensity = request.minIntensity
            # radar_spoke_msg.max_intensity = request.maxIntensity
            # radar_spoke_msg.num_samples = request.numSamples

            self.radar_pub.publish(radar_spoke_msg)

        return sensor_streaming_pb2.RadarStreamingResponse(success=True)

    def StreamImuSensor(self, request_iterator, context):
        # TODO - write this
        for request in request_iterator:
            acc = request.acceleration
            pub = self._get_publisher("imu", request.sensorId)
            pub.publish(Vector3(acc.x, acc.y, acc.z))


    def StreamPoseSensor(self, request_iterator, context):

        for request in request_iterator:
            pub = self._get_publisher("pose", request.sensorId)
            pub.publish(Pose())
            pass

    def StreamDepthSensor(self, request_iterator, context):

        for request in request_iterator:
            depth = request.depth
            pub = self._get_publisher("depth", request.sensorId)
            pub.publish(Float32(depth))