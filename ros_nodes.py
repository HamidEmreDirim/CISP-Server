import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from webcam import WebcamPublisher
from webcam_zeromq import CameraSubscriber  # Import CameraSubscriber from webcam_zeromq.py
import time

navbar_data = {'temperature': 0, 'humidity': 0, 'battery': 0, 'connection': 'waiting...'}
last_message_time = time.time()

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('UI_Sensor_Subscriber')
        self.create_subscription(String, 'temperature', self.temperature_callback, 10)
        self.create_subscription(String, 'humidity', self.humidity_callback, 10)
        self.create_subscription(String, 'battery', self.battery_callback, 10)

    def temperature_callback(self, msg):
        navbar_data['temperature'] = msg.data

    def humidity_callback(self, msg):
        navbar_data['humidity'] = msg.data

    def battery_callback(self, msg):
        navbar_data['battery'] = msg.data
        global last_message_time
        last_message_time = time.time()

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'joystickData', 10)
       
    def publish_joystick_data(self, joystick_data):
        msg = String()
        msg.data = str(joystick_data)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joystick data: {msg.data}")

def init_ros_node():
    rclpy.init()

    # Instantiate nodes
    sensor_node = SensorSubscriber()
    joystick_node = JoystickPublisher()
    webcam_node = WebcamPublisher()
    camera_subscriber_node = CameraSubscriber()  # Instantiate the CameraSubscriber node

    # Use MultiThreadedExecutor to handle multiple nodes
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(joystick_node)
    executor.add_node(webcam_node)
    executor.add_node(camera_subscriber_node)  # Add CameraSubscriber to executor

    try:
        executor.spin()  # Spin all nodes together
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        executor.shutdown()
        sensor_node.destroy_node()
        joystick_node.destroy_node()
        webcam_node.destroy_node()
        camera_subscriber_node.destroy_node()
        rclpy.shutdown()
