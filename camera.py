import zmq
import cv2
import base64
import pickle
import struct
from flask_socketio import SocketIO
import time

context = zmq.Context()
camera_socket = context.socket(zmq.SUB)
camera_socket.connect('tcp://10.116.64.102:5555')
camera_socket.setsockopt_string(zmq.SUBSCRIBE, '')

def camera_stream(socketio):
    payload_size = struct.calcsize("Q")
    data = b""
    
    while True:
        try:
            while len(data) < payload_size:
                data += camera_socket.recv()

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += camera_socket.recv()

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            image_data = f"data:image/jpeg;base64,{jpg_as_text}"

            # Emit the frame to the frontend
            socketio.emit("camera_frame", image_data)
            time.sleep(0.03)

        except Exception as e:
            print("Error in camera stream:", e)

# Function to start the camera stream
def start_camera_stream(socketio):
    socketio.start_background_task(camera_stream, socketio)
