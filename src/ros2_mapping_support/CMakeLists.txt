cmake_minimum_required(VERSION 3.8)
project(ros2_mapping_support)

# Core ROS 2 build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package (defined by setup.py / setup.cfg)
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes / scripts
install(
  PROGRAMS
    bno055_imu_node.py
    fake_odom_from_tf.py
    TabletPather.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install ALL configs (this is what fixes your error)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resources (e.g. package marker)
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
