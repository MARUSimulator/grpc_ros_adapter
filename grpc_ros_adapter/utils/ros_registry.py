import utils.ros_handle as rh
import time
from msg_translator import ProtoRosTranslator

class RosRegistry:

    _publishers = dict()

    _translator = ProtoRosTranslator()

    @classmethod
    def get_publisher(cls, topic_name, data_class, queue_size=10):
        # TODO: better check and logging

        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name

        publisher = cls._publishers.get(topic_name, None)
        if not publisher:
            publisher = rh.Publisher(data_class, topic_name, qos_profile=queue_size)
            time.sleep(0.1)
            cls._publishers[topic_name] = publisher

        if not publisher.data_class == data_class:
            rh.logerr(f"Publisher on topic {topic_name} cannot have data_class {data_class.__name__}")

        return publisher

    @classmethod
    def translate_ros2proto(cls, msg):
        return cls._translator.ros2proto(msg)

    @classmethod
    def translate_proto2ros(cls, msg):
        return cls._translator.proto2ros(msg)

    @classmethod
    def add_proto2ros_translator(cls, msg, translate):
        cls._translator.proto2ros_custom_translator[msg] = translate

    @classmethod
    def add_ros2proto_translator(cls, msg, translate):
        cls._translator.ros2proto_custom_translator[msg] = translate
    

    @classmethod
    def remap_proto2ros(cls, remap_from, remap_to):
        cls._translator.proto_remaps[remap_from] = remap_to