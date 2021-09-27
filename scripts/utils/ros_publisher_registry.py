import rospy

class RosPublisherRegistry:

    _publishers = dict()

    @classmethod
    def get_publisher(cls, topic_name, data_class, queue_size=10):
        # TODO: better check and logging

        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name

        publisher = cls._publishers.get(topic_name, None)
        if not publisher:
            publisher = rospy.Publisher(topic_name, data_class, queue_size=queue_size)
            rospy.sleep(0.2)
            cls._publishers[topic_name] = publisher

        if not publisher.data_class == data_class:
            rospy.logerr(f"Publisher on topic {topic_name} cannot have data_class {data_class.__name__}")


        return publisher