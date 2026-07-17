import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
from pyModbusTCP.client import ModbusClient
from diagnostic_msgs.msg import DiagnosticArray


class ModbusController(Node):
    """Node that bridges NDVI topics to a Modbus TCP device.

    It subscribes to ``/ndvi1`` through ``/ndvi5`` (``Float32``) and
    writes the value to coils 0-4 on a Modbus server.  Only strict
    ``0.0`` or ``1.0`` values are accepted; other numbers are ignored.

    The connection parameters are read from ``config/config.yaml`` of
    the ``fertilizer`` package under the ``modbus`` key.
    """

    def __init__(self):
        super().__init__('modbus_controller')

        # load configuration file from package's config directory
        # Try multiple locations to find config.yaml
        cfg_path = None
        
        # Try abspath based on current file location
        potential_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'config.yaml',
        ))
        if os.path.exists(potential_path):
            cfg_path = potential_path
        
        # Try home-based path
        if not cfg_path:
            home_path = os.path.expanduser('~/ros2_ws/src/fertilizer/config/config.yaml')
            if os.path.exists(home_path):
                cfg_path = home_path
        
        # Try common paths
        if not cfg_path:
            common_paths = [
                '/home/jetson/ros2_ws/src/fertilizer/config/config.yaml',
                '/root/ros2_ws/src/fertilizer/config/config.yaml',
            ]
            for path in common_paths:
                if os.path.exists(path):
                    cfg_path = path
                    break
        
        if not cfg_path:
            self.get_logger().error('Could not find config.yaml in any expected location')
            cfg = {}
        else:
            try:
                with open(cfg_path, 'r') as fh:
                    cfg = yaml.safe_load(fh)
            except Exception as e:
                self.get_logger().error(f'Failed to read config file {cfg_path}: {e}')
                cfg = {}

        modb_cfg = cfg.get('modbus', {})

        host = modb_cfg.get('host', '192.168.11.60')
        port = modb_cfg.get('port', 502)
        auto_open = modb_cfg.get('auto_open', True)
        self.threshold = cfg.get('threshold', 0)

        self.camera_map = modb_cfg.get('camera_map', {})
        if not self.camera_map:
            self.get_logger().warn(
                'No camera_map found in config.yaml, coils will not be written')

        # instantiate Modbus client; try opening once
        self.client = ModbusClient(host=host, port=port, auto_open=auto_open)
        try:
            self.client.open()
        except Exception as ex:
            self.get_logger().error(f'Unable to open Modbus connection: {ex}')

        #anti bounce to protect the valves
        self.debounce = modb_cfg.get('debounce_time', 500_000)
        self.last_change = {}
        self.last_val = {}



        self.create_subscription(
            DiagnosticArray, '/detection/Trigger', self._detection_cb, 10)

        self.get_logger().info('ModbusController: Subscribed to /detection/Trigger')

        #send true to coils 1-5 to open fertilizer flow at startup
        if self.camera_map:
            n_coils = max(self.camera_map.values())
            self.client.write_multiple_coils(0, [True] * n_coils)

        self.get_logger().info('ModbusController initialised')


    def _detection_cb(self, msg: DiagnosticArray):
        for status in msg.status:
            topic_name = status.name
            camera_name = topic_name.split('/')[1]
            coil_index = self.camera_map.get(camera_name)

            if coil_index is None:
                self.get_logger().error(
                    f'Unknown camera_name "{camera_name}", no coil mapping, skipping')
                continue

            # extract the prediction's value
            coil_val = None
            for kv in status.values:
                if kv.key == 'plant_detected':
                    coil_val = (kv.value == 'true')
                    break

            if coil_val is None:
                self.get_logger().error(
                    f'No plant_detected key for camera "{camera_name}", skipping')
                continue

            last_val = self.last_val.get(coil_index, 0)

            now_time = time.monotonic_ns()
            last_time = self.last_change.get(coil_index, 0)
            elapsed_time = (now_time - last_time)/1000

            if elapsed_time < self.debounce and last_val != coil_val :
                self.get_logger().info(f'Change didn t apply for Coil {coil_index} because the last change is too recent, last_val = {last_val}, actual_val = {coil_val}')
                continue


            try:
                self.client.write_single_coil(coil_index, coil_val)
                self.get_logger().info(
                    f'Coil {coil_index} ({camera_name}) set to {coil_val} from {last_val}')
            except Exception as ex:
                self.get_logger().error(f'Failed writing coil {coil_index}: {ex}')
                continue

            if last_val != coil_val:
                self.get_logger().info(
                    f'Coil {coil_index} ({camera_name}) set to {coil_val}')
            
            self.last_change[coil_index]=now_time
            self.last_val[coil_index]=coil_val


def main(args=None):
    rclpy.init(args=args)
    node = ModbusController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
