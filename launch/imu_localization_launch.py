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
        Node(
            package='ros2_icm20948',
            executable='imu_to_odom',
            name='imu_to_odom',
            output='screen',
            remappings=[
                ('/odom','/odom_from_imu'),
            ],

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
                {"frequency": 15.0},
                {"sensor_timeout": 1.0},
                {"two_d_mode": True},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"world_frame": "odom"},
                {"publish_tf": True},
                {"imu0": "/imu/data"},
                {"imu0_config": [False, False, False,
                                 False, False, False,
                                 False, False, False,
                                 False, False, True,
                                 True, True, False,]},
                {"imu0_differential": False},
                {"imu0_relative": False},
                {"imu0_queue_size": 5},
                {"imu0_frame_id": "imu_icm20948"},
                {"imu0_remove_gravitational_acceleration": True},
                {"odom0":"/odom_from_imu"},
                {"odom0_config": [False,False,False,
                                  False,False,False,
                                  True,True,False,
                                  False,False,True,
                                  False,False,False]},
            ],
            remappings=[
                ('/odometry/filtered','/odom'),
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_tf',
            arguments=["0","0","0","0","0","0","base_link","imu_icm20948"],
            output="screen"
        )

    ])
