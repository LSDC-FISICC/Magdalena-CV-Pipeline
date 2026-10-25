import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# message_filters provides approximate time synchronization for multiple
# subscribers.  Use it to align color/infra frames by timestamp.
from message_filters import Subscriber, ApproximateTimeSynchronizer

# cv_bridge and OpenCV are used to convert between ROS images and numpy
# arrays so we can crop a region of interest.
from cv_bridge import CvBridge
import cv2

# YAML will read configuration parameters for the ROI.
import yaml

import numpy as np
import math


def get_roi_dimensions(height_cm=140, cm_x=120, cm_y=30, scale=0.5):
    """
    Calculates ROI dimensions in pixels based on camera height and physical dimensions.
    
    Args:
        height_cm (float): Vertical distance from camera to ground.
        cm_x (float): Physical width of the ROI in cm.
        cm_y (float): Physical height of the ROI in cm.
        scale (float): Divisor for the physical-to-pixel scaling.
        
    Returns:
        tuple: (width, height) in pixels.
    """
    # 1. Define Camera Resolution
    ancho = 640
    alto = 360

    # 2. Calculate Ground Span (Trigonometry)
    # Using the confirmed angles: 45 deg (H) and 32.5 deg (V)
    # tan(45°) = 1.0, tan(32.5°) ≈ 0.637
    span_x = height_cm * math.tan(45 * math.pi / 180)   
    span_y = height_cm * math.tan(32.5 * math.pi / 180) 

    # 3. Calculate ROI Dimensions in Pixels (a = width, b = height)
    # Formula: (Resolution * (ObjectSize / Scale)) / GroundSpan
    a = int(ancho * (cm_x / (scale * height_cm)) / (span_x / height_cm))
    b = int(alto * (cm_y / (scale * height_cm)) / (span_y / height_cm))

    return a, b


class ProcessingPipeline(Node):
    """
    A node that subscribes to both color and infra image topics, crops a
    region of interest (ROI) defined in a YAML config file, and publishes
    the results on separate output topics.  The two input streams are
    approximately time-synchronized using message_filters.ApproximateTimeSynchronizer.

    The config file may also provide an optional 3×3 alignment matrix that
    warps the infrared image into the coordinate frame of the color image
    before cropping.  This makes it easy to compensate for the hardware
    offset between the RealSense color and infrared sensors without
    modifying any upstream launch files.

    Alternatively, set the `dynamic` flag under `alignment` to true and the
    node will estimate a homography on every frame via ORB feature matching
    with RANSAC; this is slower but adapts to movement/defocus.

    Topics are hardcoded here but may be remapped at launch time.
    """

    def __init__(self):
        super().__init__('processing_pipeline')

        # Read ROI parameters from config file in package's config directory.
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

        roi = cfg.get('roi', {})
        cm_x = roi.get('w', 120)
        cm_y = roi.get('h', 230)
        self.roi_w, self.roi_h = get_roi_dimensions(cm_x=cm_x, cm_y=cm_y, scale=1)
        self.roi_x = (640 - self.roi_w) // 2
        self.roi_y = 0
        self.get_logger().info(
            f'Using ROI x={self.roi_x} y={self.roi_y} w={self.roi_w} h={self.roi_h}'
        )

        # Optional alignment configuration.  The matrix should map infrared
        # coordinates into the color frame.  It can be computed offline using
        # RealSense calibration tools or from camera intrinsics/extrinsics,
        # or estimated live on each frame via feature matching (RANSAC).  See
        # config.yaml for format options.
        align_cfg = cfg.get('alignment', {})
        self.align_enabled = bool(align_cfg.get('enabled', False))
        self.align_dynamic = bool(align_cfg.get('dynamic', False))
        self.align_matrix = None
        # ORB matcher infrastructure for dynamic mode
        self.orb = None
        self.bf = None

        if self.align_enabled:
            mat = align_cfg.get('matrix', None)
            if mat is not None:
                try:
                    self.align_matrix = np.array(mat, dtype=np.float32)
                    if self.align_matrix.shape != (3, 3):
                        raise ValueError('alignment matrix must be 3x3')
                except Exception as e:
                    self.get_logger().error(
                        f'Invalid alignment matrix in config: {e} -- disabling alignment'
                    )
                    self.align_enabled = False
            elif self.align_dynamic:
                # prepare feature matcher for live estimation
                self.orb = cv2.ORB_create(nfeatures=500)
                self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                self.get_logger().info('Dynamic alignment enabled: will compute homography each frame via ORB feature matching')
                self.get_logger().info(f'ORB: {self.orb}, BFMatcher: {self.bf}')
            else:
                self.get_logger().error(
                    'Alignment enabled in config but no matrix provided; disabling'
                )
                self.align_enabled = False
        if self.align_enabled:
            mode = 'dynamic' if self.align_dynamic else 'static'
            self.get_logger().info(f'Infra→color alignment enabled ({mode})')
        else:
            self.get_logger().info('Infra→color alignment disabled')

        self.bridge = CvBridge()

        # Get topic names from parameters (set by launch file)
        self.declare_parameter('color_topic', '/color')
        self.declare_parameter('infra_topic', '/infra')
        self.declare_parameter('color_output_topic', '/output')
        self.declare_parameter('infra_output_topic', '/infra_output')

        color_topic = self.get_parameter('color_topic').value
        infra_topic = self.get_parameter('infra_topic').value
        color_output_topic = self.get_parameter('color_output_topic').value
        infra_output_topic = self.get_parameter('infra_output_topic').value

        self.get_logger().info(f'ProcessingPipeline initialized')
        self.get_logger().info(f'  Input topics:  {color_topic}, {infra_topic}')
        self.get_logger().info(f'  Output topics: {color_output_topic}, {infra_output_topic}')

        # Publishers for the cropped output images.
        self.color_pub = self.create_publisher(Image, str(color_output_topic), 10)
        self.infra_pub = self.create_publisher(Image, str(infra_output_topic), 10)

        # Set up synchronized subscriptions using message_filters.
        color_sub = Subscriber(self, Image, str(color_topic))
        infra_sub = Subscriber(self, Image, str(infra_topic))
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, infra_sub],
            queue_size=10,
            slop=0.05,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._synced_callback)

        self.get_logger().info('ProcessingPipeline initialized; listening on /color and /infra')

    def _synced_callback(self, color_msg: Image, infra_msg: Image):
        """Handles a pair of approximately time-synchronized images."""
        # convert to OpenCV images
        try:
            color_cv = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            infra_cv = self.bridge.imgmsg_to_cv2(infra_msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # optionally compute or apply alignment
        if self.align_enabled:
            # dynamic mode: estimate homography each frame
            if self.align_dynamic and self.orb is not None and self.bf is not None:
                try:
                    gray_color = cv2.cvtColor(color_cv, cv2.COLOR_BGR2GRAY)
                    kp_color, desc_color = self.orb.detectAndCompute(gray_color, None)
                    kp_infra, desc_infra = self.orb.detectAndCompute(infra_cv, None)
                    if desc_color is not None and desc_infra is not None:
                        matches = self.bf.match(desc_infra, desc_color)
                        matches = sorted(matches, key=lambda m: m.distance)
                        if len(matches) >= 10:
                            # convert to numpy arrays explicitly to satisfy type checker
                            src_pts = np.array([kp_infra[m.queryIdx].pt for m in matches], dtype=np.float32)
                            dst_pts = np.array([kp_color[m.trainIdx].pt for m in matches], dtype=np.float32)
                            src = src_pts.reshape(-1, 1, 2)
                            dst = dst_pts.reshape(-1, 1, 2)
                            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
                            if H is not None:
                                H = H / H[2, 2]
                                self.align_matrix = H
                                #self.get_logger().info(f'Computed dynamic alignment matrix with {len(matches)} matches')
                            else:
                                self.get_logger().warning('Could not compute homography from matches')
                        else:
                            self.get_logger().warning(f'Not enough matches for dynamic alignment: {len(matches)}')
                    else:
                        self.get_logger().warning('No descriptors found for dynamic alignment')
                except Exception as e:
                    self.get_logger().warning(f'Live alignment failed: {e}')
            else:
                self.get_logger().warning(f'ORB or BFMatcher not initialized for dynamic alignment. align_dynamic: {self.align_dynamic}, "orb": {self.orb is not None}, "bf": {self.bf is not None}')
            # if we have a matrix (either provided or computed) apply it
            if self.align_matrix is not None:
                try:
                    h_color, w_color = color_cv.shape[:2]
                    infra_cv = cv2.warpPerspective(
                        infra_cv,
                        self.align_matrix,
                        (w_color, h_color),
                        flags=cv2.INTER_LINEAR,
                    )
                except Exception as e:
                    self.get_logger().error(f'Alignment transform failed: {e}')
                    # continue without alignment

        # crop ROI for both images (same dimensions assumed)
        x, y, w, h = self.roi_x, self.roi_y, self.roi_w, self.roi_h
        if w > 0 and h > 0:
            color_crop = color_cv[y : y + h, x : x + w]
            infra_crop = infra_cv[y : y + h, x : x + w]
        else:
            # no cropping configured, pass through
            color_crop = color_cv
            infra_crop = infra_cv


        #filter color crop to only show green areas (vegetation)
        # cv2 hue is 0-179, saturation and value are 0-255.
        #   green is 120 +- 40 typically, so 60 +- 20 is (40, 80).
        # sat and val should be from 30% to 100%, so ~75 to 255 in 0-255 range.
        hsv_crop = cv2.cvtColor(color_crop, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 75, 75])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_crop, lower_green, upper_green)
        color_crop = cv2.bitwise_and(color_crop, color_crop, mask=mask)

        # convert back to ROS messages and publish
        try:
            out_color = self.bridge.cv2_to_imgmsg(color_crop, encoding='bgr8')
            out_infra = self.bridge.cv2_to_imgmsg(infra_crop, encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge back conversion failed: {e}')
            return

        # preserve headers (stamp/frame_id) from original messages
        out_color.header = color_msg.header
        out_infra.header = infra_msg.header

        self.color_pub.publish(out_color)
        self.infra_pub.publish(out_infra)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessingPipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
