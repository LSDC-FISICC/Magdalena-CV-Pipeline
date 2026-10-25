import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
from pyModbusTCP.client import ModbusClient
from diagnostic_msgs.msg import DiagnosticArray


class ModbusController(Node):
    """ROS 2 Node that bridges vision detection triggers to a Modbus TCP device.

    The valve decisions come from one perception pipeline, chosen by the ``source``
    parameter (default: ``valve_source`` in config.yaml):

    * ``yolo``: the ``/detection/Trigger`` topic (``DiagnosticArray``) of the YOLO26
      inference node, one status per camera with a ``plant_detected`` key;
    * ``index``: the ``/threshold<i>`` topics (``Float32``, 1.0 = plant) of the
      NDVI/colour-index nodes, ``i`` being the camera number.

    Each decision updates the Modbus coil of its camera (``camera_map``) through the
    same debounce, so both pipelines drive the valves identically.

    Connection and mapping parameters are loaded from the package's ``config.yaml``.
    """

    def __init__(self):
        super().__init__('modbus_controller')

        # ----------------------------------------------------------------------
        # Configuration File Loading
        # ----------------------------------------------------------------------
        cfg_path = None
        
        # Strategy 1: Look for config relative to the current script location
        potential_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'config.yaml',
        ))
        if os.path.exists(potential_path):
            cfg_path = potential_path
        
        # Strategy 2 : Try home-based path
        if not cfg_path:
            home_path = os.path.expanduser('~/ros2_ws/src/fertilizer/config/config.yaml')
            if os.path.exists(home_path):
                cfg_path = home_path
        
        # Strategy 3 : Try common paths
        if not cfg_path:
            common_paths = [
                '/home/jetson/ros2_ws/src/fertilizer/config/config.yaml',
                '/root/ros2_ws/src/fertilizer/config/config.yaml',
            ]
            for path in common_paths:
                if os.path.exists(path):
                    cfg_path = path
                    break

        # Parse YAML file if found, otherwise initialize empty config
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

        # ----------------------------------------------------------------------
        # Parameter Initialization
        # ----------------------------------------------------------------------

        modb_cfg = cfg.get('modbus', {})

        host = modb_cfg.get('host', '192.168.11.60')
        port = modb_cfg.get('port', 502)
        auto_open = modb_cfg.get('auto_open', True)
        self.threshold = cfg.get('threshold', 0)


        # Map camera names to Modbus coil indices
        self.camera_map = modb_cfg.get('camera_map', {})
        if not self.camera_map:
            self.get_logger().warn(
                'No camera_map found in config.yaml, coils will not be written')
            
        # ----------------------------------------------------------------------
        # Modbus Client Setup
        # ----------------------------------------------------------------------

        # instantiate Modbus client; try opening once
        self.client = ModbusClient(host=host, port=port, auto_open=auto_open)
        try:
            self.client.open()
        except Exception as ex:
            self.get_logger().error(f'Unable to open Modbus connection: {ex}')

        # Valve protection: Debounce configuration tracker (in microseconds)
        self.debounce = modb_cfg.get('debounce_time', 500_000)
        self.last_change = {}
        self.last_val = {}
        # ----------------------------------------------------------------------
        # ROS 2 Subscriptions & Initialization Actions
        # ----------------------------------------------------------------------

        self.source = self.declare_parameter('source', cfg.get('valve_source', 'yolo')).value
        if self.source == 'index':
            for camera_name in self.camera_map:
                topic = f'/threshold{camera_name.removeprefix("camera")}'
                self.create_subscription(Float32, topic, self._make_index_cb(camera_name), 10)
                self.get_logger().info(f'ModbusController: {camera_name} <- {topic}')
        else:
            self.create_subscription(
                DiagnosticArray, '/detection/Trigger', self._detection_cb, 10)
            self.get_logger().info('ModbusController: Subscribed to /detection/Trigger')

        # Start coils 0..max(camera_map) in "apply" (True) and record it, so the debounce
        # starts from the real valve state. Writing only 0..max-1 skipped the highest
        # mapped coil (camera5 -> coil 5), which started in "return".
        if self.camera_map:
            n_coils = max(self.camera_map.values()) + 1
            self.client.write_multiple_coils(0, [True] * n_coils)
            self.last_val.update({coil: True for coil in range(n_coils)})

        # Heartbeat for the PLC watchdog: the PLC releases every valve to its
        # de-energised (apply) state when this coil stops toggling.
        self.heartbeat_coil = modb_cfg.get('heartbeat_coil')
        self.heartbeat = False
        if self.heartbeat_coil is not None:
            period_s = modb_cfg.get('heartbeat_period_ms', 200) / 1000
            self.create_timer(period_s, self._heartbeat_cb)

        self.get_logger().info('ModbusController initialised')

    def _heartbeat_cb(self):
        """Toggle the heartbeat coil; a failed write is retried on the next tick."""
        self.heartbeat = not self.heartbeat
        err = ''
        try:
            # pyModbusTCP returns None/False instead of raising when the write fails
            ok = self.client.write_single_coil(self.heartbeat_coil, self.heartbeat)
        except Exception as ex:
            ok, err = False, f' ({ex})'
        if not ok:
            self.get_logger().error(f'Heartbeat coil {self.heartbeat_coil} not written{err}: the PLC '
                                    'watchdog will release the valves to apply', throttle_duration_sec=5.0)


    def _detection_cb(self, msg: DiagnosticArray):
        """YOLO decisions: one status per camera with a ``plant_detected`` key."""
        for status in msg.status:
            # Extract camera name from topic string (e.g., "/camera1/features" -> "camera1")
            camera_name = status.name.split('/')[1]
            coil_val = None
            for kv in status.values:
                if kv.key == 'plant_detected':
                    coil_val = (kv.value == 'true')
                    break
            if coil_val is None:
                self.get_logger().error(
                    f'No plant_detected key for camera "{camera_name}", skipping')
                continue
            self._set_coil(camera_name, coil_val)

    def _make_index_cb(self, camera_name):
        """Index decisions: ndvi_threshold publishes 1.0 (plant) or 0.0 per frame."""
        def cb(msg: Float32):
            self._set_coil(camera_name, msg.data > 0.5)
        return cb

    def _set_coil(self, camera_name, coil_val):
        """Write one camera's decision to its coil, honouring the debounce."""
        coil_index = self.camera_map.get(camera_name)
        if coil_index is None:
            self.get_logger().error(
                f'Unknown camera_name "{camera_name}", no coil mapping, skipping')
            return

        # ------------------------------------------------------------------
        # Debounce Logic (Hardware Protection)
        # ------------------------------------------------------------------
        last_val = self.last_val.get(coil_index, 0)

        now_time = time.monotonic_ns()
        last_time = self.last_change.get(coil_index, 0)

        # Convert elapsed time from nanoseconds to microseconds
        elapsed_time = (now_time - last_time)/1000

        # Reject requested state change if it happens faster than the debounce limit
        if elapsed_time < self.debounce and last_val != coil_val :
            self.get_logger().info(f'Change didn t apply for Coil {coil_index} because the last change is too recent, last_val = {last_val}, actual_val = {coil_val}')
            return

        # ------------------------------------------------------------------
        # Modbus Output Write
        # ------------------------------------------------------------------

        try:
            self.client.write_single_coil(coil_index, coil_val)
            self.get_logger().info(
                f'Coil {coil_index} ({camera_name}) set to {coil_val} from {last_val}')
        except Exception as ex:
            self.get_logger().error(f'Failed writing coil {coil_index}: {ex}')
            return

        if last_val != coil_val:
            self.last_change[coil_index]=now_time
            self.get_logger().info(
                f'Coil {coil_index} ({camera_name}) set to {coil_val}')

        # Update history states for the next callback execution
        self.last_val[coil_index]=coil_val


def main(args=None):
    rclpy.init(args=args)
    node = ModbusController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
