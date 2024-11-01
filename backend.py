from flask import Flask, request
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
import threading
import subprocess
import time
import zmq
import cv2
import pickle
import struct
import base64
import numpy as np
from datetime import datetime
from std_msgs.msg import String


# Flask ve SocketIO yapılandırması
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

context = zmq.Context()
camera_socket = context.socket(zmq.SUB)
camera_socket.connect('tcp://192.168.171.229:5555') 
camera_socket.setsockopt_string(zmq.SUBSCRIBE, '') 


def start_webcam_script():
    try:
        process = subprocess.Popen(
            ["python3", "webcam.py"],  # webcam.py scriptini başlatır
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd="."  # Aynı klasörde çalıştır
        )

        # webcam.py scriptinin çıktısını dinle
        for line in iter(process.stdout.readline, ""):
            print(line.strip())  # Konsola çıktıyı yazdır
            if "Published an image frame." in line:
                print("Trigger condition met. Starting webcam_zeromq.py")
                
                # Şart sağlanırsa webcam_zeromq.py çalıştırılır
                subprocess.Popen(["python3", "webcam_zeromq.py"], cwd=".")
                break  # İlk scriptten çıkış yapar
    except Exception as e:
        print("Error starting webcam script:", e)




def camera_stream():
    payload_size = struct.calcsize("Q")
    data = b""
    
    while True:
        try:
            # Mesajın veri boyutunu al
            while len(data) < payload_size:
                data += camera_socket.recv()

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            # Tam kare verisini al
            while len(data) < msg_size:
                data += camera_socket.recv()

            frame_data = data[:msg_size]
            data = data[msg_size:]

            # JPEG formatında sıkıştırılmış görüntüyü aç
            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            # JPEG olarak sıkıştır ve base64'e çevir
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            image_data = f"data:image/jpeg;base64,{jpg_as_text}"

            # Görüntüyü Socket.IO ile frontend’e gönder
            socketio.emit("camera_frame", image_data)
            time.sleep(0.03)  # Veri iletim hızını ayarlamak için

        except Exception as e:
            print("Error in camera stream:", e)






# Zaman damgası formatı
date = datetime.now()
formatted_date = date.strftime("%H:%M:%S -- %d-%m-%Y")

# Gerekli değişkenler
navbar_data = {'temperature': 0, 'humidity': 0, 'battery': 0, 'connection': 'waiting...'}
speedF = 30
joystick_data = {'x': '0', 'y': '0', 'z': '0', 'plow': '0', 'speedF': speedF, 'turnType': 0}
last_message_time = time.time()
timeout = 5  # 5 saniye

# ROS2 Node sınıfı tanımı
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('UI_Sensor_Subscriber')
        self.create_subscription(String, 'temperature', self.temperature_callback, 10)
        self.create_subscription(String, 'humidity', self.humidity_callback, 10)
        self.create_subscription(String, 'battery', self.battery_callback, 10)

    def temperature_callback(self, msg):
        navbar_data['temperature'] = msg.data
        # Dosya kaydı gibi işlemler burada yapılabilir

    def humidity_callback(self, msg):
        navbar_data['humidity'] = msg.data
        # Dosya kaydı gibi işlemler burada yapılabilir

    def battery_callback(self, msg):
        navbar_data['battery'] = msg.data
        global last_message_time
        last_message_time = time.time()
        # Dosya kaydı gibi işlemler burada yapılabilir

# ROS 2 Joystick Publisher Node
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'joystickData', 10)
        
    def publish_joystick_data(self, joystick_data):
        msg = String()
        msg.data = str(joystick_data)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joystick data: {msg.data}")

# ROS 2 düğüm başlatma fonksiyonu
def init_ros_node():
    rclpy.init()
    sensor_node = SensorSubscriber()
    joystick_node = JoystickPublisher()

    # Spin each node in separate threads
    rclpy.spin(sensor_node)
    rclpy.spin(joystick_node)

# ROS düğümünü ayrı bir iş parçacığında başlatma
ros_thread = threading.Thread(target=init_ros_node)
ros_thread.start()

# JoystickPublisher örneğini başlatma
joystick_publisher = JoystickPublisher()

# SocketIO bağlantı olayları
@socketio.on("connect")
def handle_connect():
    print("Connection established - server")
    emit("Navbar", navbar_data)

@socketio.on("Joystick")
def handle_joystick(data):
    global joystick_data
    joystick_data.update(data)
    emit("Joystick", joystick_data)
    
    # Joystick verisini ROS 2 topiğine yayınlama
    joystick_publisher.publish_joystick_data(joystick_data)

@socketio.on("Stop")
def handle_stop():
    print("Stop")
    # Dosya kaydı gibi işlemler burada yapılabilir

@socketio.on("autonomousDrive")
def handle_autonomous_drive(data):
    print(f"Autonomous Drive: {data}")
    # Dosya kaydı gibi işlemler burada yapılabilir

@socketio.on("autonomousState")
def handle_autonomous_state(data):
    print(f"Autonomous State: {data}")
    # Dosya kaydı gibi işlemler burada yapılabilir

@socketio.on("turnType")
def handle_turn_type(data):
    joystick_data['turnType'] = data
    print(f"Turn Type: {data}")

@socketio.on("cameraSelect")
def handle_camera_select(data):
    print(f"Camera Select: {data}")
    # Dosya kaydı gibi işlemler burada yapılabilir

@socketio.on("speedFactor")
def handle_speed_factor(data):
    global speedF
    speedF = data
    joystick_data['speedF'] = data
    emit("Joystick", joystick_data)

@socketio.on("Load")
def handle_load(data):
    emit("LoadUI", data)

@socketio.on("plow")
def handle_plow(data):
    joystick_data['plow'] = data
    emit("Joystick", joystick_data)

# Navbar güncelleme işlemi
def update_navbar():
    while True:
        if time.time() - last_message_time <= timeout:
            navbar_data['connection'] = 'Connected'
        socketio.emit("Navbar", navbar_data)
        time.sleep(1)


webcam_thread = threading.Thread(target=start_webcam_script)
webcam_thread.start()

# Kamera akışını ayrı bir thread ile başlat
camera_thread = threading.Thread(target=camera_stream)
camera_thread.start()


# Navbar güncelleme işlevini ayrı bir iş parçacığında çalıştırma
navbar_thread = threading.Thread(target=update_navbar)
navbar_thread.start()

# Sunucu başlatma
if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=4000)
