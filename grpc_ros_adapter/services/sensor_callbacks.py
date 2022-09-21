import inspect, sys

import numpy as np
import cv2
import utils.ros_handle as rh
import utils.extensions
from protobuf.sensor_pb2 import PointCloud2
from utils.ros_publisher_registry import RosPublisherRegistry

from google.protobuf import text_format
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix, PointCloud, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose, Quaternion, PoseWithCovarianceStamped, Point, TwistWithCovarianceStamped

def publish_image(request, context):
    if not hasattr(context, "bridge"):
        context.bridge = CvBridge()


    if len(str(request.image)) > 0:
        img_string = request.image.data
        cv_image = np.fromstring(img_string, np.uint8)

        # NOTE, the height is specifiec as a parameter before the width
        cv_image = cv_image.reshape(request.image.height, request.image.width, 3)
        cv_image = cv2.flip(cv_image, 0)

        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        msg = Image()
        header = Header()
        try:
            # RGB
            # msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

            # BGR
            msg = context.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

            header.stamp = rh.Time.from_sec(request.image.header.timestamp)
            header.frame_id = request.image.header.frameId
            msg.header = header
        except CvBridgeError as e:
            print(e)

        pub = RosPublisherRegistry.get_publisher(request.address.lower(), Image)
        pub.publish(msg)

    elif len(str(request.compressedImage)) > 0:

        msg = CompressedImage()
        header = Header()
        header.stamp = rh.Time.from_sec(request.compressedImage.header.timestamp)
        header.frame_id = request.compressedImage.header.frameId
        msg.header = header
        msg.data = request.compressedImage.data
        msg.format = request.compressedImage.format

        pub = RosPublisherRegistry.get_publisher(request.address.lower() + "/compressed", CompressedImage)
        pub.publish(msg)


def publish_sonar_image(request, context):
    msg = CompressedImage()
    header = Header()
    header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    header.frame_id = request.data.header.frameId
    msg.header = header
    msg.data = request.data.data
    msg.format = request.data.format
    pub = RosPublisherRegistry.get_publisher(request.address.lower() + "/compressed", CompressedImage)
    pub.publish(msg)

def publish_imu(request, context):
    imu = Imu()

    imu.header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    imu.header.frame_id = request.data.header.frameId
    imu.linear_acceleration = request.data.linearAcceleration.as_ros()
    imu.angular_velocity = request.data.angularVelocity.as_ros()
    imu.orientation = request.data.orientation.as_ros()

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), Imu)
    pub.publish(imu)


def publish_pose(request, context):
    from auv_msgs.msg import NavigationStatus, NED
    nav = NavigationStatus()
    pos = request.data.position
    o = request.data.orientation.as_ros()
    nav.header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    nav.header.frame_id = request.data.header.frameId
    nav.position =  NED(north=pos.north, east=pos.east, depth=pos.depth)
    nav.orientation = o
    nav.seafloor_velocity = request.data.seafloorVelocity.as_ros()
    nav.body_velocity = request.data.bodyVelocity.as_ros()
    nav.orientation_rate = request.data.orientationRate.as_ros()

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), NavigationStatus)
    pub.publish(nav)

def publish_depth(request, context):
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    pose.header.frame_id = request.data.header.frameId
    pose.pose.pose.position = Point(x=0, y=0, z=-request.data.pose.pose.position.z)
    #pose.pose.covariance = request.data.pose.covariance

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), PoseWithCovarianceStamped)
    pub.publish(pose)

def publish_dvl(request, context):

    from uuv_sensor_msgs.msg import DVL
    # not tested
    dvl = TwistWithCovarianceStamped()
    dvl.header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    dvl.header.frame_id = request.data.header.frameId
    dvl.twist.twist.linear = request.data.twist.twist.linear.as_ros()

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), TwistWithCovarianceStamped)
    pub.publish(dvl)

def publish_sonar_fix(request, context):

    from underwater_msgs.msg import SonarFix
    # not tested
    sonar = SonarFix()
    sonar.bearing = request.bearing
    sonar.range = request.range
    pub = RosPublisherRegistry.get_publisher(request.address.lower(), SonarFix)
    pub.publish(sonar)


def publish_sonar(request, context):
    pointcloud_msg = PointCloud()
    header = Header()
    header.frame_id = request.data.header.frameId
    header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    pointcloud_msg.header = header

    pointcloud_msg.points = request.data.points
    pointcloud_msg.channels = request.data.channels

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), PointCloud)
    pub.publish(pointcloud_msg)


def publish_gnss(request, context):
    geo_point = NavSatFix()
    geo_point.header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    geo_point.header.frame_id = request.data.header.frameId
    geo_point.status.status = request.data.status.status
    geo_point.status.service = request.data.status.service
    geo_point.latitude = request.data.latitude
    geo_point.longitude = request.data.longitude
    geo_point.altitude = request.data.altitude

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), NavSatFix)
    pub.publish(geo_point)
    pass

def publish_ais(request, context):
    from uuv_sensor_msgs.msg import AISPositionReport
    report = AISPositionReport()
    report.type = request.aisPositionReport.type
    report.mmsi = request.aisPositionReport.mmsi
    report.heading = request.aisPositionReport.heading
    point = NavSatFix()
    point.latitude = request.aisPositionReport.geopoint.latitude
    point.longitude = request.aisPositionReport.geopoint.longitude
    point.altitude = request.aisPositionReport.geopoint.altitude
    report.position = point
    report.speedOverGround = request.aisPositionReport.speedOverGround
    report.courseOverGround = request.aisPositionReport.courseOverGround
    pub = RosPublisherRegistry.get_publisher(request.address.lower(), AISPositionReport)
    pub.publish(report)

def publish_pointcloud(request, context):
    pointcloud_msg = PointCloud()
    header = Header()
    header.frame_id = request.data.header.frameId
    header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    pointcloud_msg.header = header

    pointcloud_msg.points = request.data.points
    pointcloud_msg.channels = request.data.channels

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), PointCloud)
    pub.publish(pointcloud_msg)

def publish_pointcloud2(request, context):
    pointcloud_msg = PointCloud2()
    header = Header()
    header.frame_id = request.data.header.frameId
    header.stamp = rh.Time.from_sec(request.data.header.timestamp)
    pointcloud_msg.header = header

    pointcloud_msg.data = request.data.data

    fields = []
    for f in request.data.fields:
        pf = PointField()
        pf.count = f.count
        pf.datatype = f.datatype
        pf.offset = f.offset
        pf.name = f.name
        fields.append(pf)
    pointcloud_msg.fields = fields
    pointcloud_msg.height = request.data.height
    pointcloud_msg.width = request.data.width
    pointcloud_msg.is_bigendian = request.data.isBigEndian
    pointcloud_msg.point_step = request.data.pointStep
    pointcloud_msg.row_step = request.data.rowStep

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), PointCloud2)
    pub.publish(pointcloud_msg)

# expose only functions
__all__ = [x[0] for x in inspect.getmembers(sys.modules[__name__], inspect.isfunction)]
