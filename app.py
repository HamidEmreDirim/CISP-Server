from flask import Flask
from flask_socketio import SocketIO
from threading import Thread
from camera import start_camera_stream
from ros_nodes import init_ros_node, JoystickPublisher
from socketio_events import socketio_events, set_joystick_publisher
from navbar import update_navbar
import atexit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Register socketio events
socketio_events(socketio)

def main():
    # Start ROS2 node thread
    ros_thread = Thread(target=init_ros_node)
    ros_thread.start()

    # Initialize the JoystickPublisher and pass it to socketio_events
    joystick_publisher = JoystickPublisher()
    set_joystick_publisher(joystick_publisher)

    # Start the camera stream after socketio initialization
    start_camera_stream(socketio)

    # Start navbar update thread
    navbar_thread = Thread(target=update_navbar)
    navbar_thread.start()

    # Gracefully shut down threads
    def cleanup():
        ros_thread.join()
        navbar_thread.join()

    atexit.register(cleanup)

    socketio.run(app, host="0.0.0.0", port=4000)

if __name__ == "__main__":
    main()
