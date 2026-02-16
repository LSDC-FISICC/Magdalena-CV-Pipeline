import rclpy
from rclpy.node import Node
import subprocess

CAMERAS = {
    "cam1" : "471300003-02"
}

class CameraCheck(Node):
    def __init__(self):
        super().__init__('camera_check')

        # Timer para revisar cada 1 segundo
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


def main(args=None):
    rclpy.init(args=args)
    node = CameraCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.archivo.close()
        node.destroy_node()
        #rclpy.shutdown()
        #cv2.destroyAllWindows()
        #GPIO.cleanup()


"""
import rclpy
from rclpy.node import Node
import subprocess

CAMS = {
    "cam1" : "471300003-02"
}

class CameraCheck(Node):
    def __init__(self):
        super().__init__('camera_check')

        self.expected_cameras = 1

        self.timer = self.create_timer(0.5, self.check_camera_alive)

    def check_camera_alive(self):
        result = subprocess.run(["lsusb", "-v", "-d", "8086:"], capture_output = True, text=True)
        lines = result.stdout.splitlines()

        intel_devices = [line for line in lines if "8086:" in line]
        found = len(intel_devices)

        for i, dev in enumerate(intel_devices, 1):
            self.get_logger().info(f"- Camara {i}: {dev}")

        if found == self.expected_cameras:
          self.get_logger().warn(f"Camara Conectada")
        else:
          self.get_logger().info(f"Camara Desconectada")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.archivo.close()
        node.destroy_node()
        #rclpy.shutdown()
        #cv2.destroyAllWindows()
        #GPIO.cleanup()

"""