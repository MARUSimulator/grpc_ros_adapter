import threading

import utils.ros_handle as rh
import time
from queue import Queue, Empty

class Streamer:

    """
    Server streaming service.
    POC implementation
    """
    def __init__(self, make_response, topic_msg_type, callbacks={}):

        # map clients to their buffers
        self._registered_clients = {}
        # map address to clients that are listening to it
        self._address_to_clients_map = {}
        self._thread_sleep_if_empty = 0.05
        self._callbacks = callbacks

        self._topic_msg_type = topic_msg_type
        self._make_response = make_response


    def _subscribe_to_topic(self, address, msg_type):
        def callback(msg, *args):
            for cl in self._address_to_clients_map[address]:
                self._registered_clients[(cl, address)].put(msg, block=False)

        rh.Subscription(msg_type, address, callback, 10)

    def start_stream(self, request, context):
        
        client_id = context.peer() 
        address = request.address.lower()
        request_buffer = Queue(100) # every client has a request buffer for every veh it controls 

        # add buffer to registered clients
        self._registered_clients[(client_id, address)] = request_buffer

        # subscribe to topic if client is not already listening to it
        if address not in self._address_to_clients_map:
            self._subscribe_to_topic(address, self._topic_msg_type)
            self._address_to_clients_map[address] = [ client_id ]
        if client_id not in self._address_to_clients_map[address]:
            self._address_to_clients_map[address].append(client_id)

        while True:
            # if connection closed
            if not context.is_active():
                self._remove_client(request, context)
                return 

            if request_buffer.empty():
                time.sleep(self._thread_sleep_if_empty)
                continue

            data = None
            try:
                data = request_buffer.get(timeout=1)
            except Empty:
                continue

            try:
                response = self._make_response(data)
            except Exception as e:
                rh.logerr(f"Cannot create response message for topic in {address}. {repr(e)}.")
                continue

            yield response

    def _remove_client(self, request, context):
        address = request.address.lower()
        client_id = context.peer()
        
        cl_list = self._address_to_clients_map[address]
        cl_list.remove(client_id)
        self._registered_clients.pop((client_id, address))

