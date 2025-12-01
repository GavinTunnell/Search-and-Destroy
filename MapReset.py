#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import time


class TabletResetRunner(Node):
    def __init__(self):
        super().__init__('tablet_reset_runner')

        # Path to your reset bash script
        self.reset_cmd = "bash ~/reset_cartographer.sh"  # <-- Edit this path if needed

        self.sub = self.create_subscription(
            Empty,
            '/tablet_reset_slam',  # Topic you're already publishing to
            self.on_reset,
            10
        )

        self.get_logger().info(
            'TabletResetRunner listening on /tablet_reset_slam (std_msgs/Empty).'
        )
        self.get_logger().info(f'Configured reset_cmd: {self.reset_cmd}')

    def on_reset(self, msg: Empty):
        self.get_logger().info(f'Reset requested – running: {self.reset_cmd}')

        # Start by killing any existing Cartographer nodes and restart them
        try:
            # Run the bash reset script in a login shell
            subprocess.Popen(
                ['bash', '-lc', self.reset_cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info('Cartographer reset command triggered.')
        except Exception as e:
            self.get_logger().error(f'Failed to run reset_cmd: {e}')

        # Optional: sleep to allow Cartographer time to restart
        time.sleep(2)

        # Additional reset logic from your script: kill Cartographer nodes
        self.get_logger().info("[reset_cartographer] Killing existing Cartographer nodes (if any)...")
        try:
            subprocess.run(['pkill', '-f', 'cartographer_ros.*cartographer_node'], check=False)
            subprocess.run(['pkill', '-f', 'cartographer_ros.*cartographer_occupancy_grid_node'], check=False)
        except Exception as e:
            self.get_logger().warn(f'Failed to kill Cartographer nodes (may be harmless): {e}')

        # Give ROS time to clean up
        time.sleep(2)

        # Get the config directory for Cartographer and restart the nodes
        try:
            config_dir = subprocess.check_output(
                "source /opt/ros/humble/setup.bash && source ~/carto_ws/install/setup.bash && ros2 pkg prefix ros2_mapping_support", 
                shell=True, stderr=subprocess.PIPE
            ).decode('utf-8').strip() + "/share/ros2_mapping_support/config"
            self.get_logger().info(f"[reset_cartographer] Using config dir: {config_dir}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to get config directory: {e}")
            self.get_logger().error(f"stderr: {e.stderr.decode('utf-8')}")
            return
        
        self.get_logger().info("[reset_cartographer] Restarting cartographer_node...")
        try:
            # Start the Cartographer node
            subprocess.Popen(
                ['bash', '-lc', f"ros2 run cartographer_ros cartographer_node -configuration_directory {config_dir} -configuration_basename cartographer_2d_no_odom.lua --ros-args -r __node:=cartographer_node -r scan:=scan -r imu:=imu/data"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("[reset_cartographer] cartographer_node restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart cartographer_node: {e}")

        time.sleep(2)

        self.get_logger().info("[reset_cartographer] Restarting occupancy_grid_node...")
        try:
            # Start the occupancy grid node (map publisher)
            subprocess.Popen(
                ['bash', '-lc', "ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -r __node:=cartographer_occupancy_grid_node -p resolution:=0.05 -p publish_period_sec:=0.3"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("[reset_cartographer] occupancy_grid_node restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart occupancy_grid_node: {e}")

        self.get_logger().info("[reset_cartographer] Cartographer reset complete, new map started.")


def main(args=None):
    rclpy.init(args=args)
    node = TabletResetRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
