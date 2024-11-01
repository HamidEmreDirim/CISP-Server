import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.cap = cv2.VideoCapture(0)  # Capture from the default webcam
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device.")
            rclpy.shutdown()
        print("Published an image frame.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert the raw OpenCV image frame to a ROS2 Image message
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(image_message)
                
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to convert image: {e}")
        else:
            self.get_logger().warn("Failed to capture an image frame.")

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
        self.get_logger().info("Released video capture device.")

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()

    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass

    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

