from std_msgs.msg import Float32MultiArray
from grpc_utils import topic_streamer
import remote_control_pb2
import remote_control_pb2_grpc
import std_pb2



class RemoteControl(remote_control_pb2_grpc.RemoteControlServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, callbacks={}):

        self._callbacks = callbacks
        self._streamer = topic_streamer.Streamer(
                RemoteControl.make_response, Float32MultiArray)

    def ApplyForce(self, request, context):

        for msg in self._streamer.start_stream(request, context):
            response = std_pb2.Float32Array()
            response.data.extend(msg.pwm.data)
            response = remote_control_pb2.ForceResponse(success=1, pwm=response)

            callbacks = self._callbacks.get("ApplyForce", [])
            for c in callbacks:
                c(response, context)

            yield response

    @staticmethod
    def make_response(pwm_out):
        response = std_pb2.Float32Array()
        response.data.extend(pwm_out.data)
        response = remote_control_pb2.ForceResponse(success=1, pwm=response)
        return response