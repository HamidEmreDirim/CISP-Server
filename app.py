from flask import Flask
from flask_socketio import SocketIO
from threading import Thread
import atexit

# ROS node fonksiyonlarımız
from ros_nodes import init_ros_node, JoystickPublisher, set_socketio
from socketio_events import socketio_events, set_joystick_publisher
from navbar import update_navbar

from terminal_pty import register_terminal_events   # ← EKLE


# ZeroMQ abone tarafı
from camera_zmq import start_camera_streams

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app,  async_mode="threading" ,cors_allowed_origins="*")


socketio_events(socketio)
register_terminal_events(socketio)  # ← EKLE


def main():
    # ROS tarafına SocketIO referansı ver
    set_socketio(socketio)

    # ROS2 node thread
    ros_thread = Thread(target=init_ros_node)
    ros_thread.start()

    # JoystickPublisher oluştur
    joystick_publisher = JoystickPublisher()
    set_joystick_publisher(joystick_publisher)

    # Navbar update thread: socketio ve navbar_data parametreleriyle çağırıyoruz.
    from ros_nodes import navbar_data  # navbar_data'ya ROS tarafından erişiyoruz.
    navbar_thread = Thread(target=update_navbar, args=(socketio, navbar_data))
    navbar_thread.start()

    # ZeroMQ -> SocketIO kamera akışlarını başlat
    start_camera_streams(socketio)

    def cleanup():
        ros_thread.join()
        navbar_thread.join()

    atexit.register(cleanup)

    socketio.run(app, host="0.0.0.0", port=4000)

if __name__ == "__main__":
    main()
