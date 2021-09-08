
import numpy as np
import grpc
import cv2

from protobuf import tf_pb2_grpc
from protobuf import tf_pb2

import rospy
from tf2_msgs.msg import TFMessage
from threading import Lock

class FrameService(tf_pb2_grpc.TfServicer):

    def __init__(self):
        self.static_tfs = []
        self.tfs = []
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/tf_static", TFMessage, self.static_tf_callback)

    def tf_callback(self, message):
        self.tfs = message.transforms

    def static_tf_callback(self, message):
        self.static_tfs = message.transforms

    def GetAllFrames(self, request, context):
        
        def create_tf_msg(tf):
            return tf_pb2.TfFrame(
                frameId=tf.header.frame_id,
                childFrameId=tf.child_frame_id,
                translation = tf.transform.translation.as_msg(),
                rotation = tf.transform.rotation.as_msg()
            )
        
        frames = []
        for tf in self.tfs:
            tf_msg = create_tf_msg(tf)
            frames.append(tf_msg)

        for tf in self.static_tfs:
            tf_msg = create_tf_msg(tf)
            frames.append(tf_msg)

        response = tf_pb2.TfFrameList()
        response.frames.extend(frames)
        return response

    def StreamAllFrames(self, request, context):
        raise NotImplementedError()
