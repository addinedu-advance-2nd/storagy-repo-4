import cv2
import socket
import pickle
import struct

ip = '192.168.0.179'
port = 50001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((ip, port))
    print("connection success")

    data_buffer = b""
    data_size = 4

    while True:
        while len(data_buffer) < data_size:
            data_buffer += client_socket.recv(4096)
        
        packed_data_size = data_buffer[:data_size]
        data_buffer = data_buffer[data_size:]

        frame_size = struct.unpack(">L", packed_data_size)[0]

        while len(data_buffer) < frame_size:
            data_buffer += client_socket.recv(4096)
        
        frame_data = data_buffer[:frame_size]
        data_buffer = data_buffer[frame_size:]
        print("received frame size : {} bytes".format(frame_size))

        frame = pickle.loads(frame_data)

        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    
    cv2.destroyAllWindows()