import utils.ros_handle as rh
from utils import topic_streamer
from utils.ros_publisher_registry import RosPublisherRegistry

# from ros_adapter.msg import NanomodemPayload, NanomodemRequest
from std_msgs.msg import Header
from labust_msgs.msg import NanomodemRange, NanomodemPayload, NanomodemRequest

from protobuf import labust_pb2
from protobuf import acoustic_transmission_pb2
from protobuf import acoustic_transmission_pb2_grpc
class AcousticTransmission(acoustic_transmission_pb2_grpc.AcousticTransmissionServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, callbacks={}):
        self._streamer = topic_streamer.Streamer(
                AcousticTransmission.make_response, NanomodemRequest)

    def StreamAcousticRequests(self, request, context):
        for msg in self._streamer.start_stream(request, context):
            yield msg

    def ReturnAcousticPayload(self, request, context):
        msgType = None
        msg = None

        if len(str(request.range)) > 0:
            msgType = NanomodemRange
            msg = NanomodemRange()
            msg.range = request.range.range
            msg.range_m = request.range.rangeM
            msg.id = request.range.id

            header = Header()
            header.frame_id = request.range.header.frameId
            header.stamp = rh.Time.from_sec(request.range.header.timestamp)
            msg.header = header

            pub_address = request.address.lower()

        elif len(str(request.payload)) > 0:
            msgType = NanomodemPayload
            msg = NanomodemPayload()
            msg.msg_type = request.payload.msgType
            msg.msg = request.payload.msg
            msg.sender_id = request.payload.senderId

            header = Header()
            header.frame_id = request.payload.header.frameId
            header.stamp = rh.Time.from_sec(request.payload.header.timestamp)
            msg.header = header

            pub_address = request.address.lower()

        pub = RosPublisherRegistry.get_publisher(pub_address, msgType)
        pub.publish(msg)

        callbacks = self._callbacks.get("ReturnAcousticPayload", [])
        for c in callbacks:
            c(request, context)

        yield acoustic_transmission_pb2.AcousticResponse(success=1)
    @staticmethod
    def make_response(nanomodem_req):
        response = labust_pb2.NanomodemRequest()
        response.reqType = nanomodem_req.req_type - 1
        response.scheduled = nanomodem_req.scheduled
        response.msg = nanomodem_req.msg
        response.id = nanomodem_req.id
        responseObj = acoustic_transmission_pb2.AcousticResponse(success=1, request=response)
        return responseObj
