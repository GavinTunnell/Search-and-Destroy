from setuptools import setup

package_name = 'ros2_mapping_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Configs (install everything we ship so launches can find them)
        ('share/' + package_name + '/config', [
            'config/cartographer_2d_no_odom.lua',
            'config/nav2_params_cartographer_slam.yaml',
            'config/nav2_params.yaml',
            'config/nav2_slam_toolbox_params.yaml',
            'config/simple_nav.xml',
            'config/slam_toolbox_online_async.yaml',
            'config/slam_toolbox_params.yaml',
            'config/slam_params.yaml',
            'config/my_map.yaml',
            'config/my_map.pgm',
            'config/ekf_imu_only.yaml',
        ]),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/cartographer_lidar_imu.launch.py',
            'launch/full_system_cartographer_nav2.launch.py',
            'launch/full_system_slamtoolbox_nav2.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team4',
    maintainer_email='gavintunnell20@gmail.com',
    description='Support nodes and configs for Cartographer + Nav2 mapping',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_imu_node = ros2_mapping_support.bno055_imu_node:main',
            'motor_driver_pca_reg_dual = ros2_mapping_support.motor_driver_pca_reg_dual:main',
            'nav_alignment_state_machine = ros2_mapping_support.nav_alignment_state_machine:main',
        ],
    },
)
