from queue import Queue, Empty
import time
import re
import threading
import rospy
from utils.ros_publisher_resistry import RosPublisherRegistry

from labust_msgs.msg import NanomodemRequest, NanomodemPayload, NanomodemRange

from protobuf import acoustic_transmission_pb2
from protobuf import acoustic_transmission_pb2_grpc

class Acoustics(acoustic_transmission_pb2_grpc.AcousticsServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, callbacks={}):
        print("initialized acoustic service")
        self._client_lock = threading.Lock()       
        self._veh_lock = threading.Lock()       
        # map clients to their buffers
        self._registered_clients = {}
        # map veh id to clients that are listening to it
        self._address_to_clients_map = {}
        self._thread_sleep_if_empty = 0.05
        self._callbacks = callbacks
        
    def _subscribe_to_veh(self, address, msg_type):
        def callback(force):
            with self._veh_lock:
                with self._client_lock:
                    for cl in self._address_to_clients_map[address]:
                        self._registered_clients[(cl, address)].put(force)
        
        rospy.Subscriber(address, msg_type, callback)

    def GetAcousticRequest(self, request, context):
        client_id = context.peer() 
        address = request.address.lower()
        request_buffer = Queue(100)

        with self._client_lock:
            self._registered_clients[(client_id, address)] = request_buffer

        with self._veh_lock:
            if address not in self._address_to_clients_map:
                self._subscribe_to_veh(address, NanomodemRequest)
                self._address_to_clients_map[address] = []
            self._address_to_clients_map[address].append(client_id)
        
        while True:
            
            # if connection closed
            if not context.is_active():
                self._cleanup(request, context)
                return

            if request_buffer.empty():
                time.sleep(self._thread_sleep_if_empty)
                continue
            
            nanomodem_req = None
            try:
                nanomodem_req = request_buffer.get(timeout=1)
            except Empty:
                continue

            response = acoustic_transmission_pb2.NanomodemRequest()
            response.req_type = nanomodem_req.req_type - 1
            response.scheduled = nanomodem_req.scheduled
            response.msg = nanomodem_req.msg
            response.id = nanomodem_req.id
            response = acoustic_transmission_pb2.AcousticResponse(success=1, request=response)
            callbacks = self._callbacks.get("GetAcousticRequest", [])
            for c in callbacks:
                c(request, context)

            yield response

    def ReturnAcousticPayload(self, request, context):
        msgType = None
        msg = None

        sender_id = re.findall("\/(\d+)", request.address.lower())[0]

        if len(str(request.range)) > 0:
            msgType = NanomodemRange
            msg = NanomodemRange()
            msg.range = request.range.range
            msg.range_m = request.range.range_m
            msg.id = request.range.id
            # publish range messages to nanomodem/xxx/range
            pub_address = "nanomodem/{}/range".format(sender_id)

        elif len(str(request.payload)) > 0:
            msgType = NanomodemPayload
            msg = NanomodemPayload()
            msg.msg_type = request.payload.msg_type
            # convert from string to integer list (ascii encoding)
            msg.msg = [ord(character) for character in request.payload.msg]
            msg.sender_id = request.payload.sender_id
            # publish payload messages to nanomodem/xxx/payload
            pub_address = "nanomodem/{}/payload".format(sender_id)

        pub = RosPublisherRegistry.get_publisher(pub_address, msgType)
        pub.publish(msg)

        callbacks = self._callbacks.get("ReturnAcousticPayload", [])
        for c in callbacks:
            c(request, context)
        
        yield acoustic_transmission_pb2.AcousticResponse(success=1)


    def _cleanup(self, request, context):
        veh_id = request.vehId
        client_id = context.peer()

        with self._veh_lock:
            cl_list = self._address_to_clients_map[veh_id]
            cl_list.remove(client_id)

        with self._client_lock:
            self._registered_clients.pop((client_id, veh_id))

