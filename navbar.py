from flask_socketio import emit
import time
import random  # Rastgele değer üretimi için
from ros_nodes import navbar_data, last_message_time

timeout = 5  # 5 saniye
start_time = time.time()  # Uygulama ilk açıldığında zamanın kaydı
IP_ADDRESS = "192.168.1.10"  # Dilediğiniz IP adresi

def update_navbar(socketio, navbar_data):
    while True:
        # ROS'tan son mesaj gelme süresi 5 saniyeyi aşmadıysa "Connected" kabul et
        if time.time() - last_message_time <= timeout:
            # ROS bağlantısı bilgisi ve IP adresi
            navbar_data['connection'] = f"Connected ({IP_ADDRESS})"
            
            # Sıcaklığı 20°C ile 28°C arasında rastgele ata
            navbar_data['temperature'] = random.randint(20, 28)
            
            # Nem değerini %30 ile %60 arasında rastgele ata
            navbar_data['humidity'] = random.randint(30, 60)
            
            # Pil değerini (30 dakika sonra %10'a düşecek şekilde) hesapla
            elapsed = time.time() - start_time  # Geçen süre (saniye)
            total_duration = 1800.0  # 30 dakika = 1800 saniye
            
            if elapsed >= total_duration:
                # 30 dakika geçmişse %10
                battery = 10
            else:
                # 30 dakika boyunca 100'den 10'a lineer azalım
                fraction = elapsed / total_duration   # 0 ile 1 arası
                battery = 100 - (fraction * 90)       # 100 -> 10 aralığı
            
            navbar_data['battery'] = int(battery)
        else:
            # 5 saniyeden uzun süredir ROS'tan mesaj gelmiyorsa
            pass
        # Anlık verileri frontend'e gönder
        socketio.emit("Navbar", navbar_data)
        time.sleep(1)
