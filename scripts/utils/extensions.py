from functools import wraps, partial
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3 
import protobuf.common_pb2 as common

def add_method(cls, name=""):
    def decorator(func, name=""):
        @wraps(func) 
        def wrapper(self, *args, **kwargs): 
            return func(self, *args, **kwargs)
        name = name or func.__name__
        setattr(cls, name, wrapper)
        # Note we are not binding func, but wrapper which accepts self but does exactly the same as func
        return func # returning func means func can still be used normally
    return partial(decorator, name=name)

_UNITY_TO_ROS_TRANSFORM = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])

_ROS_TO_UNITY_TRANSFORM = np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])

@add_method(Vector3, "as_msg")
def vector_as_msg(self):
    v = np.array(self.x, self.y, self.z, 1)
    v = _ROS_TO_UNITY_TRANSFORM.dot(v)
    return common.Vector3(v[0], v[1], v[2])

@add_method(common.Vector3, "as_ros")
def vector_as_ros(self):
    v = np.array([self.x, self.y, self.z , 1])
    v = _UNITY_TO_ROS_TRANSFORM.dot(v)
    return Vector3(v[0], v[1], v[2])