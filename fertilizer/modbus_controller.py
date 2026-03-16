import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
from pyModbusTCP.client import ModbusClient


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

        # instantiate Modbus client; try opening once
        self.client = ModbusClient(host=host, port=port, auto_open=auto_open)
        try:
            self.client.open()
        except Exception as ex:
            self.get_logger().error(f'Unable to open Modbus connection: {ex}')

        # subscribers for five NDVI topics
        for idx in range(5):
            topic = f'threshold{idx + 1}'
            self.create_subscription(
                Float32, topic, self._make_cb(idx), 10)

        # Explicitly log subscribed topics
        self.get_logger().info('ModbusController: Subscribed topics:' + 
                               ''.join([f' /threshold{idx+1}' for idx in range(5)]))
        self.get_logger().info('ModbusController initialized')

        #send true to coils 0-4 to open fertilizer flow at startup
        self.client.write_multiple_coils(0, [True] * 5)


    def _make_cb(self, coil_index: int):
        """Generate a callback bound to a particular coil index.

        The returned callback checks that the incoming message is exactly
        ``0.0`` or ``1.0`` and writes the corresponding boolean value to
        the Modbus coil.  Non‑conforming values are logged and ignored.
        """

        def cb(msg: Float32):
            """
            binarize input:
            """
            if msg.data < self.threshold:
                coil_val = False
            else:
                coil_val = True
            
            last_val = self.client.read_coils(coil_index, 1)
            try:
                # write_single_coil expects a boolean
                self.client.write_single_coil(coil_index, coil_val)
            except Exception as ex:
                self.get_logger().error(
                    f'Failed writing coil {coil_index}: {ex}')
            #log only trigger changes to avoid spamming logs
            
            if last_val is not None:
                if coil_val != last_val[0]:
                    self.get_logger().info(
                        f'Coil {coil_index} set to {coil_val} based on input {msg.data}')
            else:
                self.get_logger().error(f'Failed to read back coil {coil_index} after writing')


        return cb


def main(args=None):
    rclpy.init(args=args)
    node = ModbusController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
