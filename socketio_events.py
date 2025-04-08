from flask_socketio import emit
from ros_nodes import navbar_data

joystick_data = {'x': '0', 'y': '0', 'z': '0', 'plow': '0', 'speedF': 30, 'turnType': 0}
speedF = 30
joystick_publisher = None

def set_joystick_publisher(publisher):
    global joystick_publisher
    joystick_publisher = publisher

def socketio_events(socketio):
    @socketio.on("connect")
    def handle_connect():
        print("Connection established - server")
        emit("Navbar", navbar_data)

    @socketio.on("Joystick")
    def handle_joystick(data):
        joystick_data.update(data)
        emit("Joystick", joystick_data)
        if joystick_publisher:
            joystick_publisher.publish_joystick_data(joystick_data)

    @socketio.on("autonomousDrive")
    def handle_autonomous_drive(data):
        print(f"Autonomous Drive: {data}")

    @socketio.on("autonomousState")
    def handle_autonomous_state(data):
        print(f"Autonomous State: {data}")

    @socketio.on("turnType")
    def handle_turn_type(data):
        joystick_data['turnType'] = data
        print(f"Turn Type: {data}")

    @socketio.on("cameraSelect")
    def handle_camera_select(data):
        print(f"Camera Select: {data}")

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