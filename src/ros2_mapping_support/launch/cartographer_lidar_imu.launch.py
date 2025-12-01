#!/usr/bin/env python3
"""
Cartographer + sensors bringup:

- RPLidar + BNO055
- Static TFs to base_link
- Cartographer 2D + occupancy grid
- Optional RViz2 config (if present in this package)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package share (for configs)
    pkg_share = get_package_share_directory("ros2_mapping_support")

    # Optional RViz config; if missing, RViz starts empty
    rviz_config = os.path.join(pkg_share, "rviz_cartographer_nav2.rviz")
    rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []

    # Launch arguments
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLidar (e.g. /dev/ttyUSB0 or /dev/rplidar)",
    )
    declare_i2c_bus = DeclareLaunchArgument(
        "i2c_bus",
        default_value="7",
        description="I2C bus index for BNO055",
    )
    declare_i2c_address = DeclareLaunchArgument(
        "i2c_address",
        default_value="0x28",
        description="I2C address for BNO055 (e.g. 0x28)",
    )

    # Environment
    env_fastdds = SetEnvironmentVariable(
        name="FASTDDS_TRANSPORT_SHARED_MEM", value="off"
    )

    # RPLidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": 115200,
            "frame_id": "laser_frame",
            "inverted": False,
            "angle_compensate": True,
        }],
    )

    # IMU node (BNO055)
    imu_node = Node(
        package="ros2_mapping_support",
        executable="bno055_imu_node",
        name="bno055_imu_node",
        output="screen",
        parameters=[{
            "i2c_bus": LaunchConfiguration("i2c_bus"),
            "i2c_address": LaunchConfiguration("i2c_address"),
        }],
    )

    # Static transforms
    static_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "0.120", "0.0", "0.100",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "laser_frame",
        ],
        output="screen",
    )

    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "imu_link",
        ],
        output="screen",
    )

    # Cartographer 2D + occupancy grid
    carto_config_dir = os.path.join(pkg_share, "config")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", "cartographer_2d_no_odom.lua",
        ],
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("scan", "scan"),
            ("imu", "imu/data"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "resolution": 0.05,
            "publish_period_sec": 0.3,
        }],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
    )

    return LaunchDescription([
        env_fastdds,
        declare_lidar_port,
        declare_i2c_bus,
        declare_i2c_address,
        lidar_node,
        imu_node,
        static_laser_tf,
        static_imu_tf,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
