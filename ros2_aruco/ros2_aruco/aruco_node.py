import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf_transformations import quaternion_from_matrix
from rclpy.qos import qos_profile_sensor_data

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('marker_size', 0.15),
                                    ('aruco_dictionary_id', 'DICT_4X4_50'),
                                    ('image_topic', '/video0/image_raw'),
                                    ('camera_info_topic', '/video0/camera_info'),
                                    ('camera_frame', '')
                                ])

        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dictionary_id_name = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.get_logger().info(f'Subscribing to topics "{image_topic}" and "{camera_info_topic}"')
        self.get_logger().info(f'Marker size set to: {self.marker_size} meters')

        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.info_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.bridge = CvBridge()
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.intrinsic_mat = None
        self.distortion_coeffs = None

    def info_callback(self, msg):
        self.intrinsic_mat = np.array(msg.k).reshape([3, 3])
        self.distortion_coeffs = np.array(msg.d)
        self.get_logger().info('Camera info received and processed.')

    def image_callback(self, msg):
        if self.intrinsic_mat is None or self.distortion_coeffs is None:
            self.get_logger().warn('Camera calibration info not yet received.')
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion_coeffs)
            
            poses = PoseArray()
            markers = ArucoMarkers()

            # Set the header of the PoseArray and ArucoMarkers messages
            poses.header.stamp = msg.header.stamp
            poses.header.frame_id = self.camera_frame if self.camera_frame else msg.header.frame_id

            markers.header.stamp = msg.header.stamp
            markers.header.frame_id = self.camera_frame if self.camera_frame else msg.header.frame_id
            
            print(rvecs)
            print("------------")
            print(tvecs)
            for rvec, tvec in zip(rvecs, tvecs):
                tvec = tvec.flatten()
                rvec = rvec.flatten()
                pose = Pose()
                rot_matrix = np.eye(4)
                rot_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
                quaternion = quaternion_from_matrix(rot_matrix)

                pose.position.x = -tvec[1]
                pose.position.y = tvec[0]
                pose.position.z = tvec[2]
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]

                poses.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(int(ids[0][0]))

            self.poses_pub.publish(poses)
            self.markers_pub.publish(markers)
            self.get_logger().info('Published ArUco marker poses.')
    

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
