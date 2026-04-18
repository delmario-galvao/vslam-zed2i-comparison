import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'rectified_images': True,
            'enable_image_denoising': False,
            'enable_imu_fusion': False,
            'base_frame': 'zed_camera_center',
            'imu_frame': 'zed_imu_link',
            'camera_optical_frames': [
                'zed_left_camera_optical_frame',
                'zed_right_camera_optical_frame'
            ],
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
        }],
        remappings=[
            ('visual_slam/image_0', '/zed/zed_node/left_gray/image_rect_gray'),
            ('visual_slam/camera_info_0', '/zed/zed_node/left_gray/camera_info'),
            ('visual_slam/image_1', '/zed/zed_node/right_gray/image_rect_gray'),
            ('visual_slam/camera_info_1', '/zed/zed_node/right_gray/camera_info'),
            ('visual_slam/imu', '/zed/zed_node/imu/data'),
        ]
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([container])
