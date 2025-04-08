#!/usr/bin/env python3
import cv2
import numpy as np
import torch
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ThermalDetectionPublisher(Node):
    def __init__(self):
        super().__init__('thermal_detection_publisher')
        self.publisher_ = self.create_publisher(Image, 'thermal_detection_image', 10)
        self.bridge = CvBridge()

        # Kullanılacak cihaz: cuda veya cpu
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Kullanılan cihaz: {self.device}")

        # Özel YOLO modelini yükle
        try:
            self.model = YOLO(r"best.pt")  # Model dosyanızın yolunu kontrol edin
            self.model.to(self.device)
            self.get_logger().info("Özel model başarıyla yüklendi")
            self.get_logger().info(f"Model sınıfları: {self.model.names}")
        except Exception as e:
            self.get_logger().error(f"Model yükleme hatası: {e}")
            return

        # Termal kamera başlatma
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Kamera başlatılamadı!")
            return
        else:
            self.get_logger().info("Kamera başlatıldı")

        # Belirli aralıklarla (örneğin, 0.1 saniyede bir) frame işlemek için timer oluştur
        timer_period = 0.1  # saniye
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Kamera hatası!")
            return

        # Eğer görüntü renkliyse griye çevir
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # Termal görüntü için: üst yarıyı al, döndür ve kontrastı normalize et
        height = gray.shape[0]
        upper_half = gray[:height // 2, :]  # Üst yarıyı al
        rotated = cv2.rotate(upper_half, cv2.ROTATE_90_CLOCKWISE)
        normalized = cv2.normalize(rotated, None, 0, 255, cv2.NORM_MINMAX)
        processed = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

        # YOLO ile tespit işlemi
        try:
            results = self.model(processed, conf=0.25)  # Güven eşiği
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Koordinatları (CPU'ya taşı ve int'e çevir)
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    
                    # Güven skoru ve sınıf bilgisi
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    cls_name = self.model.names[cls]
                    
                    # Tespit kutusunu çiz (yayınlanan görüntü üzerinde işaretleme)
                    cv2.rectangle(processed, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    label = f'{cls_name} {conf:.2f}'
                    cv2.putText(processed, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    self.get_logger().info(f"Tespit: {cls_name}, Güven: {conf:.2f}")
        except Exception as e:
            self.get_logger().error(f"Tespit hatası: {e}")

        # İşlenmiş görüntüyü ROS Image mesajına dönüştür ve topic üzerinden yayınla
        try:
            img_msg = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
            self.publisher_.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"ROS Image mesajı oluşturma hatası: {e}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        # Açık pencere kalmaması için destroy_all_windows çağrısı yapılabilir
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Klavye kesintisi algılandı. Kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
