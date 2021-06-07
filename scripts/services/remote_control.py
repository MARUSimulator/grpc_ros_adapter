from queue import Queue
import time
import threading

import rospy
import geometry_msgs.msg as geomsgs
from std_msgs.msg import Float32MultiArray 

from protobuf import remote_control_pb2
from protobuf import remote_control_pb2_grpc
from protobuf import common_pb2

class RemoteControl(remote_control_pb2_grpc.RemoteControlServicer):
    """
    Server streaming service.
    POC implementation
    """
    def __init__(self):

        self._client_lock = threading.Lock()       
        self._veh_lock = threading.Lock()       
        # map clients to their buffers
        self._registered_clients = {}
        # map veh id to clients that are listening to it
        self._veh_to_clients_map = {}
        self._thread_sleep_if_empty = 0.05

    def _subscribe_to_veh(self, address):
        def callback(force):
            with self._veh_lock:
                with self._client_lock:
                    for cl in self._veh_to_clients_map[address]:
                        self._registered_clients[(cl, address)].put(force)

        # rospy.Subscriber("{0}/remote_control".format(veh_id), geomsgs.Vector3, callback)
        rospy.Subscriber("d2/pwm_out", Float32MultiArray, callback)

    def ApplyForce(self, request, context):
        
        client_id = context.peer() 
        address = request.address 
        request_buffer = Queue(100) # every client has a request buffer for every veh it controls 

        with self._client_lock:
            self._registered_clients[(client_id, address)] = request_buffer

        with self._veh_lock:
            if address not in self._veh_to_clients_map:
                self._subscribe_to_veh(address)
                self._veh_to_clients_map[address] = []
            self._veh_to_clients_map[address].append(client_id)

        while True:
            
            time.sleep(0.001)
            # if connection closed
            if not context.is_active():
                self._cleanup(request, context)
                return 

            if request_buffer.empty():
                time.sleep(self._thread_sleep_if_empty)
                continue
            
            pwm_out = request_buffer.get()
            response = common_pb2.Pwm()
            response.out.extend(pwm_out.data)

            response = remote_control_pb2.ForceResponse(success=1, pwm=response)
            yield response

    def _cleanup(self, request, context):
        veh_id = request.vehId
        client_id = context.peer()

        with self._veh_lock:
            cl_list = self._veh_to_clients_map[veh_id]
            cl_list.remove(client_id)

        with self._client_lock:
            self._registered_clients.pop((client_id, veh_id))

