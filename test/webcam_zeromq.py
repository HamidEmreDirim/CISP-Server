import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import zmq
import pickle
import struct
from cv_bridge import CvBridge

# ZeroMQ yayını için socket oluşturma
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind('tcp://*:5555')

# ROS2'den görüntü almak için ROS2 Node
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Bu, kameranın ROS2 topic'idir
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # ROS2 mesajını OpenCV formatına çeviriyoruz
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # OpenCV'deki görüntüyü sıkıştırıyoruz (JPEG formatında)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # Kaliteyi %50 olarak ayarla
        result, frame = cv2.imencode('.jpg', frame, encode_param)

        # Sıkıştırılmış kareyi ZeroMQ üzerinden gönderiyoruz
        data = pickle.dumps(frame)
        socket.send(struct.pack("Q", len(data)) + data)

def main(args=None):
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber()

    # ROS2 düğümünü başlat
    rclpy.spin(camera_subscriber)

    # ROS2 kapanışı
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

