from flask_socketio import emit
import time
from ros_nodes import navbar_data, last_message_time

timeout = 5  # 5 seconds

def update_navbar():
    while True:
        if time.time() - last_message_time <= timeout:
            navbar_data['connection'] = 'Connected'
        emit("Navbar", navbar_data)
        time.sleep(1)

