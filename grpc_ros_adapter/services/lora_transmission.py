import grpc_utils.ros_handle as rh
from grpc_utils import topic_streamer
from rf_msgs.msg import LoraMsg, Range, RangeingMsg
from protobuf import labust_pb2
from protobuf import rf_coms_pb2
from protobuf import rf_coms_pb2_grpc
from protobuf.std_pb2 import Empty

from grpc_utils.ros_publisher_registry import RosPublisherRegistry

class LoraTransmission(rf_coms_pb2_grpc.LoraTransmissionServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, callbacks={}):
        self._msg_streamer = topic_streamer.Streamer(
                RosPublisherRegistry.translate_ros2proto, LoraMsg)

    def StreamRangeingMsgs(self, request_iterator, context):
        for request in request_iterator:
            pub = RosPublisherRegistry.get_publisher(request.address.lower(), RangeingMsg)
            msg = RosPublisherRegistry.translate_proto2ros(request)
            pub.publish(msg)
        return Empty()

    def ReceiveLoraMessages(self, request, context):
        for msg in self._msg_streamer.start_stream(request, context):
            yield msg

    def SendLoraMessage(self, request, context):
        pub = RosPublisherRegistry.get_publisher(request.address.lower(), LoraMsg)
        msg = RosPublisherRegistry.translate_proto2ros(request)
        if msg:
            pub.publish(msg)
        return Empty()
