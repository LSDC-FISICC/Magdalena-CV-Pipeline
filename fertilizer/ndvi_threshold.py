import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml


class NDVIThreshold(Node):
    """
    A node that subscribes to cropped color (/roi) and infrared (/infra_roi) images,
    computes the normalized NDVI index, and publishes:
    - /threshold: Float32 (1.0 if NDVI average > threshold, else 0.0)
    - /ndvi: Image with NDVI visualization (RdYlGn-like colormap, RGB 8-bit)

    The two input streams are approximately time-synchronized using
    message_filters.ApproximateTimeSynchronizer.
    """

    def __init__(self):
        super().__init__('ndvi_threshold')

        # Read threshold parameter from config file.
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
        
        cfg = {}
        if cfg_path:
            try:
                with open(cfg_path, 'r') as f:
                    cfg = yaml.safe_load(f)
            except Exception as e:
                self.get_logger().error(f'Failed to read config file {cfg_path}: {e}')
        else:
            self.get_logger().error('Could not find config.yaml in any expected location')

        self.threshold = float(cfg.get('threshold', 0.15))
        self.get_logger().info(f'NDVI threshold set to {self.threshold}')

        self.egx_threshold = float(cfg.get('egx_threshold', 0.55))
        self.get_logger().info(f'ExG threshold set to {self.egx_threshold}')

        self.index = cfg.get('index', 'ndvi')
        self.get_logger().info(f'Index type set to {self.index}')

        self.bridge = CvBridge()

        # Get topic names from parameters (set by launch file)
        self.declare_parameter('roi_topic', '/roi')
        self.declare_parameter('infra_roi_topic', '/infra_roi')
        self.declare_parameter('threshold_output_topic', '/threshold')
        self.declare_parameter('ndvi_output_topic', '/ndvi')

        roi_topic = self.get_parameter('roi_topic').value
        infra_roi_topic = self.get_parameter('infra_roi_topic').value
        threshold_output_topic = self.get_parameter('threshold_output_topic').value
        ndvi_output_topic = self.get_parameter('ndvi_output_topic').value

        self.get_logger().info(f'NDVIThreshold initialized')
        self.get_logger().info(f'  Input topics:  {roi_topic}, {infra_roi_topic}')
        self.get_logger().info(f'  Output topics: {threshold_output_topic}, {ndvi_output_topic}')

        # Publishers.
        self.threshold_pub = self.create_publisher(Float32, str(threshold_output_topic), 10)
        self.ndvi_pub = self.create_publisher(Image, str(ndvi_output_topic), 10)

        # Set up synchronized subscriptions using message_filters.
        color_sub = Subscriber(self, Image, str(roi_topic))
        infra_sub = Subscriber(self, Image, str(infra_roi_topic))
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, infra_sub],
            queue_size=10,
            slop=0.05,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._synced_callback)

        self.get_logger().info('NDVIThreshold node initialized')

    def _synced_callback(self, color_msg: Image, infra_msg: Image):
        """Computes NDVI from synchronized color and infra images."""
        try:
            color_cv = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            infra_cv = self.bridge.imgmsg_to_cv2(infra_msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Extract channels from BGR color image (B=0, G=1, R=2).
        red = color_cv[:, :, 2].astype(np.float32)
        green = color_cv[:, :, 1].astype(np.float32)
        blue = color_cv[:, :, 0].astype(np.float32)


        if self.index == 'egx':
            # Compute ExG: 2G - R - B
            ndvi = 2 * green - red - blue
            # Normalize from (-510, 510) to (0, 1): (ndvi + 510) / 1020
            ndvi_normalized = (ndvi + 510) / 1020
            # Create binary image: 255 if pixel > threshold, else 0.
            binary = (ndvi_normalized > self.egx_threshold).astype(np.uint8) * 255
        elif self.index == 'ndvi':
            # Compute NDVI: (NIR - Red) / (NIR + Red)
            ndvi = (infra_cv.astype(np.float32) - red) / (infra_cv.astype(np.float32) + red + 1e-6)  # add small epsilon to avoid division by zero
            # Normalize from (-1, 1) to (0, 1): (ndvi + 1) / 2
            ndvi_normalized = (ndvi + 1) / 2
            #invert index
            ndvi_normalized = 1 - ndvi_normalized
            # Convert to uint8 (0-255 range) for ROS message
            binary = (ndvi_normalized * 255).astype(np.uint8)

        else:
            self.get_logger().error(f'Unknown index type: {self.index}')
            return

        # Compute average NDVI over the ROI.
        ndvi_avg = float(np.mean(binary) / 255)  # average in [0, 1]
        #self.get_logger().info(f'INDEX average: {ndvi_avg:.4f}')

        # Publish threshold result: 1.0 if average NDVI > threshold, else 0.0
        #result = 1.0 if ndvi_avg > self.threshold else 0.0
        #threshold_msg = Float32(data=result)

        # For monitoring, publish the average NDVI instead of binary result
        threshold_msg = Float32(data=ndvi_avg)  
        
        self.threshold_pub.publish(threshold_msg)


        # Convert to ROS message and publish.
        try:
            ndvi_img_msg = self.bridge.cv2_to_imgmsg(binary, encoding='mono8')
            ndvi_img_msg.header = color_msg.header
            self.ndvi_pub.publish(ndvi_img_msg)
        except Exception as e:
            self.get_logger().error(f'cv_bridge back conversion failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NDVIThreshold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
