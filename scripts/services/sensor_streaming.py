
import numpy as np
import grpc
import cv2


import utils.extensions
from protobuf import sensor_streaming_pb2
from protobuf import sensor_streaming_pb2_grpc


class Context:
    pass

class SensorStreaming(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self, callbacks={}):
        print("creating")
        self.publishers = {}
        self._callbacks = callbacks
        self._callback_contexts = {}


    def _get_callback_context(self, callback):
        context = self._callback_contexts.get(callback, None)
        if not context:
            context = Context()
        self._callback_contexts[callback] = context
        return context


    def trigger_callbacks(self, service_function, request):
        callbacks = self._callbacks.get(service_function.__name__, [])
        for c in callbacks:
            context = self._get_callback_context(c)
            c(request, context)



    def StreamCameraSensor(self, request_iterator, context):
        """
        Takes in a gRPC SensorStreamingRequest containing
        all the data needed to create and publish a sensor_msgs/Image
        ROS message.
        """
        for request in request_iterator:
            self.trigger_callbacks(self.StreamCameraSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    # def StreamLidarSensor(self, request, context):
    #     """
    #     Takes in a gRPC LidarStreamingRequest containing
    #     all the data needed to create and publish a PointCloud2
    #     ROS message.
    #     """
    #     # TODO
    #     pointcloud_msg = PointCloud2()
    #     header = std_msgs.msg.Header()
    #     header.stamp = rospy.Time.from_sec(request.timeInSeconds)

    #     header.frame_id = "velodyne"
    #     pointcloud_msg.header = header

    #     pointcloud_msg.height = request.height
    #     pointcloud_msg.width = request.width

    #     fields = request.fields

    #     # Set PointCloud[] fields in pointcloud_msg
    #     for i in range(len(fields)):
    #         pointcloud_msg.fields.append(PointField())
    #         pointcloud_msg.fields[i].name = fields[i].name
    #         pointcloud_msg.fields[i].offset = fields[i].offset
    #         pointcloud_msg.fields[i].datatype = fields[i].datatype
    #         pointcloud_msg.fields[i].count = fields[i].count

    #     pointcloud_msg.is_bigendian = request.isBigEndian
    #     pointcloud_msg.point_step = request.point_step
    #     pointcloud_msg.row_step = request.row_step

    #     pointcloud_msg.data = request.data

    #     pointcloud_msg.is_dense = request.is_dense

    #     self.lidar_pub.publish(pointcloud_msg)

    #     # TODO: This does not belong in this RPC implementation, should be
    #     # moved to own or something like that.
    #     sim_clock = Clock()
    #     sim_clock.clock = rospy.Time.from_sec(request.timeInSeconds)
    #     self.clock_pub.publish(sim_clock)

    #     return sensor_streaming_pb2.LidarStreamingResponse(success=True)

    # def StreamRadarSensor(self, request, context):
    #     """
    #     Takes in a gRPC RadarStreamingRequest containing
    #     all the data needed to create and publish a RadarSpoke
    #     ROS message.
    #     """
        
    #     # TODO
    #     number_of_spokes = request.numSpokes

    #     for i in range(number_of_spokes):

    #         radar_spoke_msg = RadarSpoke()

    #         # Header
    #         header = std_msgs.msg.Header()
    #         header.frame_id = "milliampere_radar"
    #         header.stamp = rospy.Time.from_sec(request.timeInSeconds[i])
    #         # radar_spoke_msg.azimuth = request.azimuth[i]
    #         # radar_spoke_msg.intensity = request.radarSpokes[i * request.numSamples : i * request.numSamples + request.numSamples]

    #         # radar_spoke_msg.range_start = request.rangeStart
    #         # radar_spoke_msg.range_increment = request.rangeIncrement
    #         # radar_spoke_msg.min_intensity = request.minIntensity
    #         # radar_spoke_msg.max_intensity = request.maxIntensity
    #         # radar_spoke_msg.num_samples = request.numSamples

    #         self.radar_pub.publish(radar_spoke_msg)

    #     return sensor_streaming_pb2.RadarStreamingResponse(success=True)

    def StreamImuSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamImuSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)


    def StreamPoseSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamPoseSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamDepthSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamDepthSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamDvlSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamDvlSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamSonarSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamSonarSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamGnssSensor(self, request_iterator, context):
        for request in request_iterator:
            self.trigger_callbacks(self.StreamGnssSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)

    def StreamAisSensor(self, request_iterator, context):
        
        for request in request_iterator:
            self.trigger_callbacks(self.StreamAisSensor, request)

        return sensor_streaming_pb2.StreamingResponse(success=True)