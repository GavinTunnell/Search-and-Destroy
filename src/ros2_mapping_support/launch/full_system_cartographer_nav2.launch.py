#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Launch arguments ----
    lidar_port  = LaunchConfiguration("lidar_port")
    i2c_bus     = LaunchConfiguration("i2c_bus")
    i2c_address = LaunchConfiguration("i2c_address")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_params = LaunchConfiguration("nav2_params")

    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="RPLidar serial port (e.g. /dev/ttyUSB0)",
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
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("ros2_mapping_support"),
            "config",
            "default.rviz",
        ),
        description="Full path to an RViz2 config file",
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(
            get_package_share_directory("ros2_mapping_support"),
            "config",
            "nav2_params_cartographer_slam.yaml",
        ),
        description="Nav2 params file for Cartographer SLAM/AMCL run",
    )

    # Turn off shared memory for FastDDS (needed for your motor driver)
    disable_shm = SetEnvironmentVariable(
        name="FASTDDS_TRANSPORT_SHARED_MEM",
        value="off",
    )

    pkg_share = get_package_share_directory("ros2_mapping_support")

    # ---- RPLidar node ----
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": lidar_port,
            "serial_baudrate": 115200,
            "frame_id": "laser_frame",
            "inverted": False,
            "angle_compensate": True,
        }],
    )

    # ---- IMU node (BNO055, IMUPLUS mode, no mag) ----
    imu_node = Node(
        package="ros2_mapping_support",
        executable="bno055_imu_node",
        name="bno055_imu_node",
        output="screen",
        parameters=[{
            "i2c_bus": i2c_bus,
            "i2c_address": i2c_address,
        }],
    )

    # ---- Static transforms ----
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

    # ---- Cartographer 2D + occupancy grid ----
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

    # ---- Nav2 bringup (use AMCL + map server with Cartographer map) ----
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch",
            "navigation_launch.py",
        )
    ),
    launch_arguments={
        "use_sim_time": "false",
        "autostart": "true",
        "params_file": nav2_params,
        "slam": "False",
    }.items(),
)

    # ---- Motor driver ----
    motor_driver_node = Node(
        package="ros2_mapping_support",
        executable="motor_driver_pca_reg_dual",
        name="motor_driver",
        output="screen",
        parameters=[{
            "ena_addr": "0x41",
            "enb_addr": "0x60",
            "ena_channel": 0,
            "in1_channel": 1,
            "in2_channel": 2,
            "enb_channel": 0,
            "in3_channel": 1,
            "in4_channel": 2,
            "pwm_freq_hz": 1000.0,
            "max_lin": 1.1,
            "max_ang_cmd": 1.5,
            "deadband": 0.03,
            "min_duty_pct": 55.0,
            "brake_on_zero": False,
            "invert_right": True,
            "invert_left": False,
            "map_enA_to_left": True,
        }],
    )

    # ---- RViz2 ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        disable_shm,
        declare_lidar_port,
        declare_i2c_bus,
        declare_i2c_address,
        declare_rviz_config,
        declare_nav2_params,
        lidar_node,
        imu_node,
        static_laser_tf,
        static_imu_tf,
        cartographer_node,
        occupancy_grid_node,
        nav2_bringup_launch,
        motor_driver_node,
        rviz_node,
    ])
