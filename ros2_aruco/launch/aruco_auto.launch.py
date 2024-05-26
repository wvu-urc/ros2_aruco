from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node 0
    aruco_node_0 = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_0',
        #namespace='video0',
        parameters=[
            {'image_topic': '/logitech_01/image_raw'},
            {'camera_info_topic': '/logitech_01/camera_info'}
        ],
        output='screen'
    )

    # Node 1
    aruco_node_1 = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_1',
        #namespace='camera_1',
        parameters=[
            {'image_topic': '/logitech_18/image_raw'},
            {'camera_info_topic': '/logitech_18/camera_info'}
        ],
        output='screen'
    )

    # Node 2
    aruco_node_2 = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_2',
        #namespace='camera_2',
        parameters=[
            {'image_topic': '/logitech_19/image_raw'},
            {'camera_info_topic': '/logitech_19/camera_info'}
        ],
        output='screen'
    )

    # Node 3
    aruco_node_3 = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_3',
        #namespace='camera_3',
        parameters=[
            {'image_topic': '/logitech_03/image_raw'},
            {'camera_info_topic': '/logitech_03/camera_info'}
        ],
        output='screen'
    )

    return LaunchDescription([
        aruco_node_0,
        aruco_node_1,
        aruco_node_2,
        aruco_node_3
    ])
