import zmq
import cv2
import base64
import pickle
import struct
import time

# Global ZeroMQ context (tek context, birden fazla soket için)
context = zmq.Context()

def camera_stream_rgb(socketio):
    rgb_socket = context.socket(zmq.SUB)
    try:
        rgb_socket.connect('tcp://127.0.0.1:5555')
        rgb_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        print("[DEBUG] Connected to RGB ZeroMQ publisher at tcp://127.0.0.1:5555")
    except Exception as e:
        print(f"[DEBUG] Error connecting to RGB ZeroMQ publisher: {e}")
        return

    payload_size = struct.calcsize("Q")
    data = b""

    while True:
        try:
            while len(data) < payload_size:
                data += rgb_socket.recv()
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            print(f"[DEBUG] Expected RGB frame size: {msg_size} bytes")

            while len(data) < msg_size:
                data += rgb_socket.recv()
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            if frame is None:
                print("[DEBUG] Decoded RGB frame is None")
            else:
                print("[DEBUG] RGB frame decoded successfully.")

            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            image_data = f"data:image/jpeg;base64,{jpg_as_text}"

            socketio.emit("camera_frame", image_data)
            print("[DEBUG] Emitted RGB frame to SocketIO.")
            time.sleep(0.03)
        except Exception as e:
            print("[DEBUG] Error in camera_stream_rgb:", e)
            break

def camera_stream_thermal(socketio):
    thermal_socket = context.socket(zmq.SUB)
    try:
        thermal_socket.connect('tcp://127.0.0.1:5556')
        thermal_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        print("[DEBUG] Connected to Thermal ZeroMQ publisher at tcp://127.0.0.1:5556")
    except Exception as e:
        print(f"[DEBUG] Error connecting to Thermal ZeroMQ publisher: {e}")
        return

    payload_size = struct.calcsize("Q")
    data = b""

    while True:
        try:
            while len(data) < payload_size:
                data += thermal_socket.recv()
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            print(f"[DEBUG] Expected Thermal frame size: {msg_size} bytes")

            while len(data) < msg_size:
                data += thermal_socket.recv()
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            if frame is None:
                print("[DEBUG] Decoded Thermal frame is None")
            else:
                print("[DEBUG] Thermal frame decoded successfully.")

            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            image_data = f"data:image/jpeg;base64,{jpg_as_text}"

            socketio.emit("thermal_camera_frame", image_data)
            print("[DEBUG] Emitted Thermal frame to SocketIO.")
            time.sleep(0.03)
        except Exception as e:
            print("[DEBUG] Error in camera_stream_thermal:", e)
            break

def start_camera_streams(socketio):
    socketio.start_background_task(camera_stream_rgb, socketio)
    socketio.start_background_task(camera_stream_thermal, socketio)
    print("[DEBUG] Started background tasks for camera streams.")
