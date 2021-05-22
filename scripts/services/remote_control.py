from queue import Queue
import time
import threading

import rospy
import geometry_msgs.msg as geomsgs

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

    def _subscribe_to_veh(self, veh_id):
        def callback(force):
            with self._veh_lock:
                with self._client_lock:
                    for cl in self._veh_to_clients_map[veh_id]:
                        self._registered_clients[(cl, veh_id)].put(force)

        rospy.Subscriber("{0}/remote_control".format(veh_id), geomsgs.Vector3, callback)

    def ApplyForce(self, request, context):
        
        client_id = context.peer() 
        veh_id = request.vehId
        request_buffer = Queue(100) # every client has a request buffer for every veh it controls 

        with self._client_lock:
            self._registered_clients[(client_id, veh_id)] = request_buffer

        with self._veh_lock:
            if veh_id not in self._veh_to_clients_map:
                self._subscribe_to_veh(veh_id)
                self._veh_to_clients_map[veh_id] = []
            self._veh_to_clients_map[veh_id].append(client_id)

        while True:
            
            # if connection closed
            if not context.is_active():
                self._cleanup(request, context)
                return 

            if request_buffer.empty():
                time.sleep(self._thread_sleep_if_empty)
                continue
            
            force = request_buffer.get()
            response = common_pb2.GeneralizedForce(
                x = force.x,
                y = force.y,
                z = force.z
            )

            response = remote_control_pb2.ForceResponse(success=1, generalizedForce=response)
            yield response

    def _cleanup(self, request, context):
        veh_id = request.vehId
        client_id = context.peer()

        with self._veh_lock:
            cl_list = self._veh_to_clients_map[veh_id]
            cl_list.remove(client_id)

        with self._client_lock:
            self._registered_clients.pop((client_id, veh_id))

