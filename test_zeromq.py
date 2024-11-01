import cv2
import zmq
import pickle
import struct

# ZeroMQ bağlantısı için socket oluşturma
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect('tcp://192.168.171.229:5555')  # Yayın yapan cihazın IP'si
socket.setsockopt_string(zmq.SUBSCRIBE, '')

# Veri boyutu hesaplaması
payload_size = struct.calcsize("Q")

data = b""
cv2.namedWindow("Alınan Görüntü", cv2.WINDOW_NORMAL)
while True:
    while len(data) < payload_size:
        # Veri alımı eksik olursa tamamlamak için bekler
        data += socket.recv()

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]

    msg_size = struct.unpack("Q", packed_msg_size)[0]

    while len(data) < msg_size:
        data += socket.recv()

    frame_data = data[:msg_size]
    data = data[msg_size:]

    # JPEG formatında sıkıştırılmış görüntüyü aç
    frame = pickle.loads(frame_data)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    
    if frame is not None:
        # Görüntüyü tek bir pencerede göster
        cv2.imshow("Alınan Görüntü", frame)

    # 'q' tuşu ile çıkış
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
