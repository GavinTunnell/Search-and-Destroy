mkdir -p ~/Desktop/mapping_cartogropher

cat > ~/Desktop/mapping_cartogropher/run_cartographer.sh <<'EOF'
#!/usr/bin/env bash
set -e

# Source ROS 2 and your Cartographer workspace
source /opt/ros/humble/setup.bash
source ~/carto_ws/install/setup.bash

# Launch Cartographer with RPLIDAR and BNO055 IMU
ros2 launch ros2_mapping_support cartographer_lidar_imu.launch.py \
  lidar_port:=/dev/rplidar \
  i2c_bus:=7 \
  i2c_address:=0x28
EOF

chmod +x ~/Desktop/mapping_cartogropher/run_cartographer.sh
