from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ton driver IMU
        Node(
            package='ros2_icm20948',
            executable='icm20948_node',
            name='icm20948_node',
            parameters=[
                {"i2c_address": 0x69},
                {"frame_id": "imu_icm20948"},
                {"pub_rate": 50},
            ],
            output='screen'
        ),

        # Filtre Madgwick
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{"use_mag": True}, {"gain": 0.1}, {"zeta": 0.0}],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/mag', '/imu/mag_raw'),
                ('/imu/data', '/imu/data')
            ]
        ),

        # EK robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                {"frequency": 30.0},
                {"sensor_timeout": 0.2},
                {"two_d_mode": True},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_link_frame": "imu_icm20948"},
                {"world_frame": "odom"},
                {"publish_tf": True},
                {"imu0": "/imu/data"},
                {"imu0_config": [False, False, False,
                                 True, True, True,
                                 False, False, False]},
                {"imu0_differential": False},
                {"imu0_relative": False},
                {"imu0_queue_size": 5},
                {"imu0_remove_gravitational_acceleration": True}
            ],
            output='screen'
        ),

    ])
