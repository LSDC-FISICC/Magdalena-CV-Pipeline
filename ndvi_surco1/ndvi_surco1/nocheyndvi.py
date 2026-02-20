"""@package docstring
The computer vision pipeline for NDVI generation in agricultural monitoring.
This node synchronizes RGB and infrared image streams, processes them to compute the NDVI index, and publishes the results for further analysis.
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
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm
import numpy as np
import math
import os
import time
import signal
#import Jetson.GPIO as GPIO
import subprocess
import shutil
from datetime import datetime
from datetime import datetime, timezone
import yaml
#import serial
import subprocess

led_R = 40
led_G = 38
led_B = 37
mi = 1
mrgb = 2
led_pin = 12
led_pin2 = 16
button_pin = 15
threshold = 0.4
puerto = '/dev/ttyACM0'#'COM5'  #  '/dev/ttyUSB0'
velocidad = 115200
#ser = serial.Serial(puerto, velocidad, timeout=1)
CAMERAS = {
    "cam1" : "471300003-02"
}

def cargar_configuracion(archivo):
    with open(archivo, "r") as f:
        return yaml.safe_load(f)

config = cargar_configuracion("/home/jetson/ros2_ws/src/Magdalena---Pipeline/ndvi_surco1/ndvi_surco1/config.yaml")

def image_roi(image, roi):
    """
    Applies a perspective transform to extract the region of interest (ROI) from the image.

    Args:
      image: Input image.
      roi: Coordinates of the region of interest.
    Returns:
      warped_img: The transformed image focusing on the ROI.
    """
    x = int(roi[0,0,0])
    y = int(roi[0,0,1])
    y1 = int(roi[0,2,1])
    x1 = int(roi[0,2,0])

    return image[y:y1, x:x1]





def maska(pimage):
    """
    Applies a mask to filter pixel values within a specific range.
    
    Args:
      pimage: Input processed image.
    Returns:
      mask_mean: Normalized mean of the masked image.
    """
    lower_bound = np.array([35, 40, 100])
    upper_bound = np.array([85, 255, 255])
    hsv_image = cv2.cvtColor(pimage, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    # mask_mean = np.mean(mask) / 255.0
    
    return mask





def promed(pimage,mask):
    """
    Applies a mask to filter pixel values within a specific range.
    
    Args:
      pimage: Input processed image.
    Returns:
      mask_mean: Normalized mean of the masked image.
    """
    segmented_image = cv2.bitwise_and(pimage, pimage, mask=mask)
    mask_mean = np.mean(segmented_image)
    return mask_mean 



def ndvi_generator3(bgr,infra):
    """
    Computes the NDVI (Normalized Difference Vegetation Index) from RGB and infrared images.
    Args:
      bgr: RGB image in BGR format.
      infra: Infrared image.
    Returns:
      NDVI-based masked image.
    """
    rgb_image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    red_channel = rgb_image[:, :, 0].astype(np.float32)
    nir_channel = infra.astype(np.float32)
    denominator = (nir_channel + red_channel + 1e-6)
    ndvi = (nir_channel - red_channel) / denominator
    ndvi_normalized = (ndvi + 1) / 2
    cmap = cm.get_cmap('RdYlGn')
    ndvi_colored = cmap(ndvi_normalized)
    ndvi_colored = (ndvi_colored[:, :, :3] * 255).astype(np.uint8)
    ndvi_normalized = ndvi_normalized.astype(np.uint8)
    ndvi_colored_bgr = cv2.cvtColor(ndvi_colored, cv2.COLOR_RGB2BGR)
    return ndvi_colored_bgr






class MultiTopicSync(Node):
    def __init__(self):
        super().__init__('multi_topic_sync')
        self.bridge = CvBridge()
        self.declare_parameter('camera_rgb_topic', 'image')
        self.declare_parameter('camera_infra_topic', 'image2')
        self.declare_parameter('data_topic', 'out')
        self.declare_parameter('data_topic2', 'out2')
        self.declare_parameter('ad', 10)
        camera_rgb_topic  = self.get_parameter('camera_rgb_topic').get_parameter_value().string_value
        camera_infra_topic  = self.get_parameter('camera_infra_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('data_topic').get_parameter_value().string_value
        out_topic2 = self.get_parameter('data_topic2').get_parameter_value().string_value
        self.ad = self.get_parameter('ad').get_parameter_value().integer_value
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=1,durability=DurabilityPolicy.VOLATILE)
        #
        #
        self.rgb_subscriber = Subscriber(self, Image, camera_rgb_topic, qos_profile=sensor_qos)
        self.infra_subscriber = Subscriber(self, Image, camera_infra_topic, qos_profile=sensor_qos)
        #self.rgb_subscriber = Subscriber(self, Image, camera_rgb_topic)
        #self.infra_subscriber = Subscriber(self, Image, camera_infra_topic)
        self.publisher = self.create_publisher(Image, out_topic, 10)
        self.state_publisher = self.create_publisher(Float32, out_topic2, 10)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.infra_subscriber],
            queue_size=30,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback) 
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #filename = f"CameraderechaColor_{timestamp}.txt"
        #self.archivo = open(filename, "x")
        
        self.ad=int(self.ad)
        
        #print(f"Archivo creado: {filename}")
        
        ###PRUEBA CON lsusb

        self.timer = self.create_timer(1.0, self.check_all_cameras)

    def get_connected_serials(self):
        #Obtiene todos los seriales de dispositivos Intel RealSense usando lsusb.
        try:
            result = subprocess.run(
                ["lsusb", "-v", "-d", "8086:"],
                capture_output=True,
                text=True
            )
            lines = result.stdout.splitlines()
            serials = []
            #print(lines)
            for line in lines:
                line = line.strip()
                if line.startswith("iSerial"):
                    parts = line.split()
                    serial = parts[-1]
                    serials.append(serial)
            return serials

        except Exception as e:
            self.get_logger().error(f"Error ejecutando lsusb {e}")
            return []

    def check_all_cameras(self):
        connected_serials = self.get_connected_serials()

        for cam_name, serial in CAMERAS.items():
            if serial in connected_serials:
                self.get_logger().info(f"{cam_name} conectada (serial {serial})")
            else:
                self.get_logger().error(f"{cam_name} desconectada")



        #PRUEBA PARA REVISAR SI LA CAMARA ESTA CONECTADA
        '''
        self.last_msg_time = time.time()
        self.timeout_time = 2.0
        self.timer = self.create_timer(0.5, self.check_camera_alive)

    def check_camera_alive(self):
        elapsed = time.time() - self.last_msg_time
        if elapsed > self.timeout_time:
          self.get_logger().warn(f"Camara Desconectada")
        else:
          self.get_logger().info(f"Camara Funcionando")
        '''




    def synced_callback(self, rgb_msg,infra_msg):
        """
        Processes synchronized RGB and infrared images, calculates NDVI,
        and publishes the results.

        Args:
          rgb_msg: Synchronized RGB image message.
          infra_msg: Synchronized infrared image message. 
        Returns:
          None
        """
        
        self.last_msg_time = time.time() ##CAMARA DESCONECTADA
        try:
            
            
            roix= 120#cm
            roiy= 30 #cm

            ancho = 640 
            alto = 120
             

            x = 1.2*math.tan(45*math.pi/180)   
            y = 1.2*math.tan(32.5*math.pi/180) 

            a = int(640*(roix/(4*100))/x)   
            b = int(480*(roiy/(4*100))/y)  

            the_vertices_30cm = np.array([[ (ancho/2-a, alto/2-b),
                               (ancho/2+a,alto/2-b),
                               (ancho/2+a,alto/2+b),
                               (ancho/2-a,alto/2+b)]], dtype=np.int32)

            the_vertices_1 = the_vertices_30cm

            bgr_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8'), cv2.COLOR_RGB2BGR)
            infra_image = self.bridge.imgmsg_to_cv2(infra_msg, desired_encoding='passthrough')
      
            aligned_infra = infra_image
            


            pimage1 = image_roi(bgr_image,the_vertices_1)
            pimage1infra = image_roi(aligned_infra,the_vertices_1)
            mascara = maska(pimage1)
            
            ndviSOLO= ndvi_generator3(pimage1,pimage1infra)
            
           
            
            promedio = promed(ndviSOLO,mascara)
            
            state_msg = Float32()
            state_msg.data = promedio
            
            analogica = state_msg
            #self.get_logger().info(f"promedio: {state_msg.data}")
            if state_msg.data < config["threshold"]:
              
              #ser.write(b'OFF\r')
              state_msg.data = 0.0
              #GPIO.output(led_pin, GPIO.HIGH)
              
              #self.archivo.write("0,"+ str(dt.strftime("%Y-%m-%d %H:%M:%S.%f")) + "," +str(self.fps) +"\n")
              cv2.polylines(bgr_image, [the_vertices_30cm], isClosed=True, color=(0, 0, 255), thickness=5)
            
            

            else:
              #ser.write(b'ON\r')
              state_msg.data = 1.0
              #GPIO.output(led_pin, GPIO.LOW)
              self.get_logger().info(f'Iniciado con ID de dispositivo: {self.ad}')
             
              #self.archivo.write("1,"+ str(dt.strftime("%Y-%m-%d %H:%M:%S.%f")) + "," +str(self.fps)+"\n")
              cv2.polylines(bgr_image, [the_vertices_30cm], isClosed=True, color=(0, 255,0), thickness=5)

            ndvi_msg = self.bridge.cv2_to_imgmsg(bgr_image,encoding='bgr8')
            
            self.publisher.publish(ndvi_msg)
            self.state_publisher.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f"Error calculando NDVI: {e}")

def main(args=None):
    #GPIO.setmode(GPIO.BOARD)
    #GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(led_pin2, GPIO.OUT, initial=GPIO.LOW)
    rclpy.init(args=args)
    node = MultiTopicSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.archivo.close()
        node.destroy_node()
        #rclpy.shutdown()
        cv2.destroyAllWindows()
        #GPIO.cleanup()


