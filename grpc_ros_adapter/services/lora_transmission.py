import utils.ros_handle as rh
from utils import topic_streamer
from mbzirc_msgs.msg import LoraMsg, Range, RangeingMsg
from protobuf import labust_pb2
from protobuf import rf_coms_pb2
from protobuf import rf_coms_pb2_grpc
from protobuf.std_pb2 import Empty

from utils.ros_registry import RosRegistry

class LoraTransmission(rf_coms_pb2_grpc.LoraTransmissionServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, callbacks={}):
        self._msg_streamer = topic_streamer.Streamer(
                RosRegistry.translate_ros2proto, LoraMsg)

    def StreamRangeingMsgs(self, request_iterator, context):
        for request in request_iterator:
            pub = RosRegistry.get_publisher(request.address.lower(), RangeingMsg)
            msg = RosRegistry.translate_proto2ros(request)
            pub.publish(msg)
        return Empty()

    def ReceiveLoraMessages(self, request, context):
        for msg in self._msg_streamer.start_stream(request, context):
            yield msg

    def SendLoraMessage(self, request, context):
        pub = RosRegistry.get_publisher(request.address.lower(), LoraMsg)
        msg = RosRegistry.translate_proto2ros(request)
        pub.publish(msg)
        return Empty()
