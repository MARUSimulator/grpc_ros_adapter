import numpy as np
import cv2
import rospy
from utils.ros_publisher_resistry import RosPublisherRegistry

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose, Quaternion, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import NavSatFix
from auv_msgs.msg import NavigationStatus, NED
from underwater_msgs.msg import SonarFix
from uuv_sensor_msgs.msg import DVL
from tf.transformations import quaternion_from_euler

def publish_image(request, context):

    if not hasattr(context, "bridge"):
        context.bridge = CvBridge()

    img_string = request.data
    cv_image = np.fromstring(img_string, np.uint8)

    # NOTE, the height is specifiec as a parameter before the width
    cv_image = cv_image.reshape(request.height, request.width, 3)
    cv_image = cv2.flip(cv_image, 0)

    bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

    msg = Image()
    header = Header()
    try:
        # RGB
        # msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')

        # BGR
        msg = context.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

        header.stamp = rospy.Time.from_sec(request.timeStamp)
        msg.header = header
    except CvBridgeError as e:
        print(e)

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), Image)
    pub.publish(msg)


def publish_imu(request, context):

    imu = Imu()

    imu.linear_acceleration = request.acceleration.as_ros()
    imu.angular_velocity = request.angularVelocity.as_ros()
    eu = request.orientation.as_ros()
    q = quaternion_from_euler(eu.x, eu.y, eu.z)
    imu.orientation = Quaternion(*q)

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), Imu)
    pub.publish(imu)


def publish_pose(request, context):

    nav = NavigationStatus()
    pos = request.pose.position.as_ros()
    o = request.pose.orientation.as_ros()
    nav.position = NED(pos.x, pos.y, pos.z)
    nav.orientation = o

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), NavigationStatus)
    pub.publish(nav)

def publish_depth(request, context):

    depth = request.depth
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position = Point(0, 0, -depth)

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), PoseWithCovarianceStamped)
    pub.publish(pose)

def publish_dvl(request, context):
    # not tested
    dvl = DVL()
    dvl.velocity = request.groundVelocity.as_ros()
    dvl.altitude = request.altitude
    dvl.beam_ranges.extend(request.beamRanges)

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), DVL)
    pub.publish(dvl)

def publish_sonar(request, context):
    # not tested
    sonar = SonarFix()
    sonar.bearing = request.bearing
    sonar.range = request.range

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), SonarFix)
    pub.publish(sonar)


def publish_gnss(request, context):
    geo_point = NavSatFix()
    geo_point.latitude = request.point.latitude
    geo_point.longitude = request.point.longitude
    geo_point.altitude = request.point.altitude

    pub = RosPublisherRegistry.get_publisher(request.address.lower(), NavSatFix)
    pub.publish(geo_point)
    pass