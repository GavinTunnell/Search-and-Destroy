To Run download the carto_ws zip and extract on desktop.

You also need to meet the following prequistes

NVIDIA Jetson Orin Nano with JetPack 6.2 installed.
RPLIDAR A1M8 connected via USB (/dev/ttyUSB0).
BNO055 IMU connected via I2C (pins 3/5).
Arducam IMX519 camera (CSI port).
3S LiPo battery (11.1V 5200mAh) with optional short-circuit protection
Raspberry Pi 5 with touchscreen (for wireless nav input)
PCA9685 I2C boards (3x: for motors/servos, addresses 0x41, 0x60, 0x42).
USB/I2C cables, power supplies.

Python 3.10
ROS 2 Humble
colcon build tools
rosdep
Git
Ultralytics YOLOv8 (8.2.0)
OpenCV (4.8.1.78)
NumPy (1.24.3)
TensorRT
Jetson.GPIO
smbus
Cartographer ROS
RPLIDAR ROS
Navigation2
Rosbridge Suite
tf2_ros
Nav2 Bringup
RViz2
i2c-tools
PyTorch

#Terminal 1

pkill -f nav2_bringup || true
pkill -f controller_server || true
pkill -f planner_server || true
pkill -f bt_navigator || true
pkill -f amcl || true
pkill -f map_server || true

#Terminal 2

cd ~/carto_ws
colcon build --symlink-install
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_mapping_support cartographer_lidar_imu.launch.py \
  lidar_port:=/dev/ttyUSB0 i2c_bus:=7 i2c_address:=0x28

#Terminal 3

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo map base_link

#Terminal 4

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False autostart:=True slam:=False use_composition:=False \
  params_file:=/home/team4/carto_ws/src/ros2_mapping_support/config/nav2_params_cartographer_slam_resolved.yaml \
  map:=/home/team4/carto_ws/src/ros2_mapping_support/config/my_map.yaml

#Terminal 5

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ros2_mapping_support motor_driver_pca_reg_dual --ros-args \
  -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
  -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
  -p enb_channel:=0 -p in3_channel:=1 -p in4_channel:=2 \
  -p pwm_freq_hz:=1000.0 \
  -p max_lin:=0.8 -p max_ang_cmd:=1.2 \
  -p deadband:=0.03 -p min_duty_pct:=80.0 \
  -p brake_on_zero:=false \
  -p invert_right:=true -p invert_left:=false \
  -p map_enA_to_left:=true \
  -p i2c_bus:=1 \
  -p cmd_topic:=/cmd_vel

#Terminal 6

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tablet_nav2.py
