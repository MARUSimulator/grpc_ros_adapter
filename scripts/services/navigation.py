
import rospy
import std_msgs.msg
import geometry_msgs.msg as geomsgs

from protobuf import navigation_pb2
from protobuf import navigation_pb2_grpc


class Navigation(navigation_pb2_grpc.NavigationServicer):
    def __init__(self, pose_pub, twist_pub, tf_pub):
        self.pose_pub = pose_pub
        self.twist_pub = twist_pub
        self.tf_pub = tf_pub

    def SendNavigationMessage(self, request, context):

        # TODO: This frame_id should be dynamically set from a config file.
        nav_header = std_msgs.msg.Header(
            frame_id="fosenkaia_NED",
            stamp=rospy.Time.from_sec(request.timeStamp)
        )
    
        position = geomsgs.Point()
        position.x = request.position.x
        position.y = request.position.y
        position.z = request.position.z

        orientation = geomsgs.Quaternion()
        orientation.x = request.orientation.x
        orientation.y = request.orientation.y
        orientation.z = request.orientation.z
        orientation.w = request.orientation.w

        pose_msg = geomsgs.PoseStamped(
            header=nav_header,
            pose=geomsgs.Pose(
                position=position,
                orientation=orientation
            )
        )

        self.pose_pub.publish(pose_msg)

        linear_vel = geomsgs.Vector3()
        linear_vel.x = request.linearVelocity.x
        linear_vel.y = request.linearVelocity.y
        linear_vel.z = request.linearVelocity.z

        angular_vel = geomsgs.Vector3()
        angular_vel.x = request.angularVelocity.x
        angular_vel.y = request.angularVelocity.y
        angular_vel.z = request.angularVelocity.z

        twist_msg = geomsgs.TwistStamped(
            header=nav_header,
            twist=geomsgs.Twist(
                linear=linear_vel,
                angular=angular_vel
            )
        )

        self.twist_pub.publish(twist_msg)

        transform = geomsgs.TransformStamped(
            header=nav_header,
            child_frame_id="vessel_center",
            transform=geomsgs.Transform(
                translation=position,
                rotation=orientation
            )
        )

        self.tf_pub.sendTransform(transform)

        return navigation_pb2.NavigationResponse(success=True)
