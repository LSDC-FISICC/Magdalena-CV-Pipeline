#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyModbusTCP.client import ModbusClient
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from pyModbusTCP.client import ModbusClient

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')

        
        self.declare_parameter('sensor1_topic', '/camera55/estado')
        self.declare_parameter('sensor2_topic', '/camera56/estado')
        self.declare_parameter('sensor3_topic', '/camera57/estado')
        self.declare_parameter('sensor4_topic', '/camera58/estado')
        self.declare_parameter('sensor5_topic', '/camera59/estado')

        self.declare_parameter('sensor1_topic1', '/camera55/on')
        self.declare_parameter('sensor2_topic2', '/camera56/on')
        self.declare_parameter('sensor3_topic3', '/camera57/on')
        self.declare_parameter('sensor4_topic4', '/camera58/on')
        self.declare_parameter('sensor5_topic5', '/camera59/on')

        self.declare_parameter('output_topic', '/central_data')
        self.declare_parameter('publish_period_ms', 1)  # 10 ms por defecto
        # topic ='/camera55/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) > 0:
        #     self.get_logger().info(f' está publicando en {topic}')
        # else:
        #     self.get_logger().warn(f' NO está publicando en {topic}')
        sensor1_topic = self.get_parameter(
            'sensor1_topic').get_parameter_value().string_value
        sensor2_topic = self.get_parameter(
            'sensor2_topic').get_parameter_value().string_value
        sensor3_topic = self.get_parameter(
            'sensor3_topic').get_parameter_value().string_value
        sensor4_topic = self.get_parameter(
            'sensor4_topic').get_parameter_value().string_value
        sensor5_topic = self.get_parameter(
            'sensor5_topic').get_parameter_value().string_value
        
        sensor1_topic1 = self.get_parameter(
            'sensor1_topic1').get_parameter_value().string_value
        sensor2_topic2 = self.get_parameter(
            'sensor2_topic2').get_parameter_value().string_value
        sensor3_topic3 = self.get_parameter(
            'sensor3_topic3').get_parameter_value().string_value
        sensor4_topic4 = self.get_parameter(
            'sensor4_topic4').get_parameter_value().string_value
        sensor5_topic5 = self.get_parameter(
            'sensor5_topic5').get_parameter_value().string_value
        
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        publish_period_ms = self.get_parameter(
            'publish_period_ms').get_parameter_value().double_value
        self.client = ModbusClient(host="192.168.11.60", port=502, auto_open=True)
       
        self.last_sensor1 = None
        self.last_sensor2 = None
        self.last_sensor3 = None
        self.last_sensor4 = None
        self.last_sensor5 = None

        self.last_sensor11 = None
        self.last_sensor22 = None
        self.last_sensor33 = None
        self.last_sensor44 = None
        self.last_sensor55 = None

        
        self.sensor1_sub = self.create_subscription(
            Float32,
            sensor1_topic,
            self.sensor1_callback,
            10 
        )

        self.sensor2_sub = self.create_subscription(
            Float32,
            sensor2_topic,
            self.sensor2_callback,
            10
        )
        self.sensor3_sub = self.create_subscription(
            Float32,
            sensor3_topic,
            self.sensor3_callback,
            10 
        )

        self.sensor4_sub = self.create_subscription(
            Float32,
            sensor4_topic,
            self.sensor4_callback,
            10
        )

        self.sensor5_sub = self.create_subscription(
            Float32,
            sensor5_topic,
            self.sensor5_callback,
            10
        )

        self.sensor1_sub1 = self.create_subscription(
            Float32,
            sensor1_topic1,
            self.sensor1_callback1,
            10 
        )

        self.sensor2_sub2 = self.create_subscription(
            Float32,
            sensor2_topic2,
            self.sensor2_callback2,
            10
        )
        self.sensor3_sub3 = self.create_subscription(
            Float32,
            sensor3_topic3,
            self.sensor3_callback3,
            10 
        )

        self.sensor4_sub4 = self.create_subscription(
            Float32,
            sensor4_topic4,
            self.sensor4_callback4,
            10
        )

        self.sensor5_sub5 = self.create_subscription(
            Float32,
            sensor5_topic5,
            self.sensor5_callback5,
            10
        )
        
        self.pub = self.create_publisher(
            Vector3,
            output_topic,
            10
        )

        
        timer_period_sec = publish_period_ms / 1000.0
        self.timer = self.create_timer(timer_period_sec, self.timer_callback)

        self.get_logger().info(
            f'CentralNode listo.\n'
            f'  Subscribiéndose a: {sensor1_topic}, {sensor2_topic}, {sensor3_topic}, {sensor4_topic}, {sensor5_topic}\n'
            f'  Publicando en:     {output_topic}\n'
            f'  Periodo:           {publish_period_ms} ms'
        )

    
    def sensor1_callback(self, msg: Float32):
        self.last_sensor1 = msg.data
        self.client.write_single_coil(0,  float(self.last_sensor1))

    def sensor2_callback(self, msg: Float32):
        self.last_sensor2 = msg.data
        self.client.write_single_coil(1,  float(self.last_sensor2))
    def sensor3_callback(self, msg: Float32):
        self.last_sensor3 = msg.data
        self.client.write_single_coil(2,  float(self.last_sensor3))

    def sensor4_callback(self, msg: Float32):
        self.last_sensor4 = msg.data
        self.client.write_single_coil(3,  float(self.last_sensor4))

    def sensor5_callback(self, msg: Float32):
        self.last_sensor5 = msg.data
        self.client.write_single_coil(4,  float(self.last_sensor5))



    def sensor1_callback1(self, msg: Float32):
        self.last_sensor11 = msg.data
        if self.last_sensor11 ==1.0:
            self.get_logger().warn(f"Camara 55 Desconectada")
            self.client.write_single_coil(0,  1)

    def sensor2_callback2(self, msg: Float32):
        self.last_sensor22 = msg.data
        if self.last_sensor22 ==1.0:
            self.get_logger().warn(f"Camara 56 Desconectada")
            self.client.write_single_coil(1,  1)
    def sensor3_callback3(self, msg: Float32):
        self.last_sensor33 = msg.data
        if self.last_sensor33 ==1.0:
            self.get_logger().warn(f"Camara 57 Desconectada")
            self.client.write_single_coil(2,  1)

    def sensor4_callback4(self, msg: Float32):
        self.last_sensor44 = msg.data
        if self.last_sensor44 ==1.0:
            self.get_logger().warn(f"Camara 58 Desconectada")
            self.client.write_single_coil(3, 1)

    def sensor5_callback5(self, msg: Float32):
        self.last_sensor55 = msg.data
        if self.last_sensor55 ==1.0:
            self.get_logger().warn(f"Camara 59 Desconectada")
            self.client.write_single_coil(4,  1)       

    
    def timer_callback(self):
        
        # if self.last_sensor1 is None:
        #     self.client.write_single_coil(0,  1)
        # if self.last_sensor2 is None:
        #     self.client.write_single_coil(1, 1)
        # if self.last_sensor3 is None:
        #     self.client.write_single_coil(2, 1)
        # if self.last_sensor4 is None:
        #     self.client.write_single_coil(3, 1)
        # if self.last_sensor5 is None:
        #     self.client.write_single_coil(4, 1)

        return
        # topic ='/camera55/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) <= 0:
        #     self.client.write_single_coil(0,  1)
        #     self.get_logger().warn(f"Camara 55 Desconectada")
        # topic ='/camera56/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) <= 0:
        #     self.client.write_single_coil(1,  1)
        #     self.get_logger().warn(f"Camara 56 Desconectada")
        # topic ='/camera57/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) <= 0:
        #     self.client.write_single_coil(2,  1)
        #     self.get_logger().warn(f"Camara 57 Desconectada")
        # topic ='/camera58/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) <= 0:
        #     self.client.write_single_coil(3,  1)
        #     self.get_logger().warn(f"Camara 58 Desconectada")
        # topic ='/camera59/estado'
        # info = self.get_publishers_info_by_topic(topic)
        # if len(info) <= 0:
        #     self.client.write_single_coil(4,  1)
        #     self.get_logger().warn(f"Camara 59 Desconectada")
        # out_msg = Vector3()
        # out_msg.x = float(self.last_sensor1)
        # out_msg.y = float(self.last_sensor2)
        # out_msg.z = 0.0  

        # self.pub.publish(out_msg)
        # self.get_logger().info(
        #     f'Publicando central_data: x={out_msg.x}, y={out_msg.y}'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
