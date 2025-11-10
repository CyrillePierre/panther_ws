import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

from pyproj import CRS, Transformer
import yaml
import sys
import os

class PreprocessNode(Node):
    '''
    A ROS2 node that looks up the transform between 'FP_ECEF' and 'FP_ENU0' frames,
    and adapts path_following files accordingly.

    2 blockers : 
    - wsg84 anchor point needs to be the same as .traj file 'origin'
    - localisation romea package is not used, it is replaced by Fixposition output : 
      The odometry message from FP_ECEF to FP_ENU0.
      Therefore, the anchor point must be the same as the wsg84 anchor point.

    This node :
     0. waits for TF to be available,
     1. looks up the transform between 'FP_ECEF' and 'FP_ENU0' frames,
     2. computes the FP_ENU0 corresponding LLA coordinates,
     3. modifies the 'wgs84_anchor'file accordingly to ensure consistency,
     4. modifies the '.traj' file to update the origin. 
        (using the romea_path_tools package. 
        ex: ros2 run romea_path_tools convert <input.traj> <output.traj> -a <latitude> <longitude> <altitude>)

    '''

    def __init__(self):
        super().__init__('path_following_preprocess_node')
        # Declare parameters (default values are optional)
        self.declare_parameter('demo_config_directory', '')
        self.declare_parameter('robot_config_directory', '')
        self.declare_parameter('trajectory_filename', '')

        self._done = False

        # Retrieve parameters
        self.robot_config_dir = self.get_parameter('robot_config_directory').get_parameter_value().string_value
        self.traj_file = self.get_parameter('trajectory_filename').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Waiting for TF FP_ECEF -> FP_ENU0 (10s timeout)...')
        self._tf_timer = self.create_timer(0.5, self._try_lookup_tf)

    def _try_lookup_tf(self): 
        try:
            self.transform = self.tf_buffer.lookup_transform(
                'FP_ECEF',
                'FP_ENU0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
        except Exception as e:
            self.get_logger().info('TF not available yet, retrying...')
            return

        self.get_logger().info('TF between FP_ECEF and FP_ENU0 is now available.')
        self._process_once()

    def _process_once(self):
        # Stop the timer to avoid repeated processing
        self.destroy_timer(self._tf_timer)

        # Extract translation components
        translation = self.transform.transform.translation
        x_ecef = translation.x
        y_ecef = translation.y
        z_ecef = translation.z

        self.get_logger().info(f'Obtained Transform: ECEF({x_ecef}, {y_ecef}, {z_ecef})')
        self.enu_lla = self._ecef_to_lla(x_ecef, y_ecef, z_ecef)

        # Here, you would add the logic to modify the wgs84_anchor file
        # and the .traj file using the obtained transform.
        # This is a placeholder for demonstration purposes.
        self.robot_config_dir = self.get_parameter('robot_config_directory').get_parameter_value().string_value
        self.traj_file = self.get_parameter('trajectory_filename').get_parameter_value().string_value
        self.get_logger().info('Modifying wgs84_anchor and .traj files accordingly...')
        self.get_logger().info(f'looking at {self.robot_config_dir}/wgs84_anchor.yaml and {self.robot_config_dir}paths/{self.traj_file}...')
        
        anchor_path = f'{self.robot_config_dir}/../wgs84_anchor.yaml'
        self._write_lla_to_wgs84_anchor(anchor_path)

        traj_path_input = f'{self.robot_config_dir}/../paths/{self.traj_file}'
        traj_path_output = f'{self.robot_config_dir}/../paths/{self.traj_file}'
        lat, lon, alt = self.enu_lla
        self._convert_traj_file(traj_path_input, traj_path_output, lat, lon, alt)

        # Shut down this node so launch can react to process exit
        self.get_logger().info('Preprocessing complete, shutting down and exiting.')
        self._done = True

    def _ecef_to_lla(self, x, y, z):
        transformer = Transformer.from_crs(
            CRS.from_epsg(4978),  # ECEF
            CRS.from_epsg(4979),  # LLA
            always_xy=True
        )
        lon, lat, alt = transformer.transform(x, y, z, radians=False)
        self.get_logger().info(f'Converted ECEF to LLA: Latitude={round(lat, 5)}, Longitude={round(lon, 5)}, Altitude={round(alt, 2)}')
        return lat, lon, alt
    
    def _write_lla_to_wgs84_anchor(self, path):
        lat, lon, alt = self.enu_lla

        with open(path, 'w') as file:
            yaml.dump({
                'latitude': lat,
                'longitude': lon,
                'altitude': alt
            }, file)

    def _convert_traj_file(self, input_path, output_path, lat, lon, alt):
        import subprocess
        subprocess.run([
            'ros2', 'run', 'romea_path_tools', 'convert',
            input_path,
            output_path,
            '-a', str(lat), str(lon), str(alt),
            '-f'
        ], check=True)


def main(args=None):
    rclpy.init(args=args)

    preprocess_node = PreprocessNode()

    while rclpy.ok() and not preprocess_node._done:
        rclpy.spin_once(preprocess_node)

    preprocess_node.get_logger().info('Shutting down preprocess node...')
    preprocess_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

