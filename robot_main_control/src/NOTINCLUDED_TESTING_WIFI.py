import socket
import time

TCP_IP = '192.168.0.18'
TCP_PORT = 80
BUFFER_SIZE = 5
MESSAGE = "three way\n"
data = ""

# while True:
#    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#    s.connect((TCP_IP, TCP_PORT))
#    s.send(MESSAGE.encode())
#    time.sleep(4)
#    data = s.recv(BUFFER_SIZE).decode()
#    s.close()
#    print ("received data:", data)



while True:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    data = s.recv(BUFFER_SIZE).decode().strip()
    print ("received data:", data)

    if data == "Begin":
        print("received data:", data)
        s.send(MESSAGE.encode())

    if data == "close":
        break
    
    data = ""
    time.sleep(1)
    




