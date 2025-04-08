from flask import Flask, jsonify, Response
from flask_cors import CORS  # CORS desteği için ekledik
import base64, json, gzip
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

app = Flask(__name__)
CORS(app)  # Tüm endpoint'lere CORS başlıklarını ekle
latest_pointcloud = None  # Global değişken: en son gelen point cloud verisi

def convert_pointcloud2_to_dict(msg: PointCloud2) -> dict:
    """
    ROS2 PointCloud2 mesajını, client'ların JSON olarak kullanabileceği formata dönüştürür.
    """
    header = {
        "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
        "frame_id": msg.header.frame_id
    }
    fields = []
    for field in msg.fields:
        fields.append({
            "name": field.name,
            "offset": field.offset,
            "datatype": field.datatype,
            "count": field.count
        })
    # msg.data genellikle bir dizi int ya da bytes; bytes'a çeviriyoruz
    data_bytes = bytes(msg.data) if isinstance(msg.data, (list, tuple)) else msg.data
    encoded_data = base64.b64encode(data_bytes).decode('utf-8')
    
    pointcloud_dict = {
        "header": header,
        "height": msg.height,
        "width": msg.width,
        "fields": fields,
        "is_bigendian": msg.is_bigendian,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "data": encoded_data,
        "is_dense": msg.is_dense
    }
    return pointcloud_dict

class ROS2PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('ros2_pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_map',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: PointCloud2):
        global latest_pointcloud
        latest_pointcloud = convert_pointcloud2_to_dict(msg)
        self.get_logger().info("Received /cloud_map message; latest point cloud updated.")

@app.route('/pointcloud')
def get_pointcloud():
    if latest_pointcloud is None:
        return jsonify({"error": "No point cloud data available."}), 404
    # JSON string'e çevirip gzip ile sıkıştırıyoruz
    json_str = json.dumps(latest_pointcloud)
    compressed = gzip.compress(json_str.encode('utf-8'))
    response = Response(compressed, mimetype='application/json')
    response.headers['Content-Encoding'] = 'gzip'
    return response

def ros2_thread():
    rclpy.init(args=None)
    node = ROS2PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros2_thread_obj = threading.Thread(target=ros2_thread, daemon=True)
    ros2_thread_obj.start()
    app.run(host='0.0.0.0', port=5000)
