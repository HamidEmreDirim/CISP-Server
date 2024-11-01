import threading
import subprocess
import os

def start_webcam_script():
    try:
        # Python scriptini anında yazdırılacak şekilde başlatma
        process = subprocess.Popen(
            ["python3", "webcam.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,  # Satır satır çıktıları almak için tampon boyutu 1 olarak ayarlanır
            cwd=".",
            env=dict(os.environ, PYTHONUNBUFFERED="1")  # Anında yazdırma
        )

        # webcam.py scriptinin çıktısını dinle
        for line in iter(process.stdout.readline, ""):
            print(line.strip())  # Konsola çıktıyı yazdır
            if "Published an image frame." in line:
                print("Trigger condition met. Starting webcam_zeromq.py")
                
                # Şart sağlanırsa webcam_zeromq.py çalıştırılır
                subprocess.Popen(["python3", "webcam_zeromq.py"], cwd=".")
                  # İlk scriptten çıkış yapar
    except Exception as e:
        print("Error starting webcam script:", e)

# Yeni bir iş parçacığında başlat
webcam_thread = threading.Thread(target=start_webcam_script)
webcam_thread.start()
