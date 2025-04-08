import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# ZeroMQ importları
import zmq
import pickle
import struct

navbar_data = {'temperature': 0, 'humidity': 0, 'battery': 0, 'connection': 'connected'}
last_message_time = time.time()

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('UI_Sensor_Subscriber')
        self.create_subscription(String, 'temperature', self.temperature_callback, 10)
        self.create_subscription(String, 'humidity', self.humidity_callback, 10)
        self.create_subscription(String, 'battery', self.battery_callback, 10)
        navbar_data['connection'] = 'connected'
    def temperature_callback(self, msg):
        navbar_data['temperature'] = msg.data

    def humidity_callback(self, msg):
        navbar_data['humidity'] = msg.data

    def battery_callback(self, msg):
        navbar_data['battery'] = msg.data
          # Indicate connection is alive

        global last_message_time
        last_message_time = time.time()

# SocketIO referansını tutmak için (varsa diğer yerlerde kullanmak için)
ROS_SOCKETIO = None
def set_socketio(socketio_instance):
    global ROS_SOCKETIO
    ROS_SOCKETIO = socketio_instance

class CameraSubscriber(Node):
    """
    RGB kamera verisini /rgb/image topic'inden alır,
    JPEG'e encode edip, ZeroMQ PUB üzerinden tcp://*:5555 adresine gönderir.
    """
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/rgb/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

        # ZeroMQ publisher ayarları: self.zmq_context adını kullanalım
        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.PUB)
            self.socket.bind('tcp://*:5555')
            print("[DEBUG] RGB ZeroMQ Publisher bound to tcp://*:5555")
        except Exception as e:
            print(f"[DEBUG] Error binding RGB ZeroMQ Publisher: {e}")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            if not result:
                print("[DEBUG] Failed to encode RGB frame.")
                return
            data = pickle.dumps(frame)
            # Mesaj boyutu bilgisi ile gönderiyoruz
            self.socket.send(struct.pack("Q", len(data)) + data)
            print(f"[DEBUG] RGB frame sent: {len(data)} bytes.")
        except Exception as e:
            print(f"[DEBUG] Error in RGB listener_callback: {e}")

class ThermalCameraSubscriber(Node):
    """
    Termal kamera verisini /thermal_detection_image topic'inden alır,
    JPEG'e encode edip, ZeroMQ PUB üzerinden tcp://*:5556 adresine gönderir.
    """
    def __init__(self):
        super().__init__('thermal_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/thermal_detection_image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.PUB)
            self.socket.bind('tcp://*:5556')
            print("[DEBUG] Thermal ZeroMQ Publisher bound to tcp://*:5556")
        except Exception as e:
            print(f"[DEBUG] Error binding Thermal ZeroMQ Publisher: {e}")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            if not result:
                print("[DEBUG] Failed to encode thermal frame.")
                return
            data = pickle.dumps(frame)
            self.socket.send(struct.pack("Q", len(data)) + data)
            print(f"[DEBUG] Thermal frame sent: {len(data)} bytes.")
        except Exception as e:
            print(f"[DEBUG] Error in Thermal listener_callback: {e}")

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed_pub = self.create_publisher(Int32, 'speed', 10)
        self.plow_pub = self.create_publisher(Int32, 'servo_control', 10)

    def publish_joystick_data(self, joystick_data):
        twist_msg = Twist()
        twist_msg.linear.x = float(joystick_data['x'])
        twist_msg.linear.y = float(joystick_data['y'])
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(joystick_data['z'])

        speed_msg = Int32()
        speed_msg.data = int(joystick_data['speedF'])

        plow_msg = Int32()
        plow_msg.data = int(joystick_data['plow'])

        self.twist_pub.publish(twist_msg)
        self.speed_pub.publish(speed_msg)
        self.plow_pub.publish(plow_msg)

        self.get_logger().info(
            "Published Twist: linear=({:.2f}, {:.2f}, {:.2f}), angular=({:.2f}, {:.2f}, {:.2f})".format(
                twist_msg.linear.x,
                twist_msg.linear.y,
                twist_msg.linear.z,
                twist_msg.angular.x,
                twist_msg.angular.y,
                twist_msg.angular.z
            )
        )
        self.get_logger().info(f"Published Speed: {speed_msg.data}")
        self.get_logger().info(f"Published Plow: {plow_msg.data}")

def init_ros_node():
    rclpy.init()

    camera_node = CameraSubscriber()
    sensor_node = SensorSubscriber()
    joystick_node = JoystickPublisher()
    thermal_node = ThermalCameraSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(joystick_node)
    executor.add_node(camera_node)
    executor.add_node(thermal_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("[DEBUG] KeyboardInterrupt received, shutting down ROS nodes.")
    finally:
        executor.shutdown()
        sensor_node.destroy_node()
        joystick_node.destroy_node()
        camera_node.destroy_node()
        thermal_node.destroy_node()
        rclpy.shutdown()
