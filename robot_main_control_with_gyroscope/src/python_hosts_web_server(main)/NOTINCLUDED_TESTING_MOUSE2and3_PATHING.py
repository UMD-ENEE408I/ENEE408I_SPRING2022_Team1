import socket
import time

BUFFER_SIZE = 5
MESSAGE = ""
data = ""
string_traversal_path = "Right,Right,Forward,Right,Left,Right,Right,Right,Forward,Left,WINNER\n"


SERVER_HOST = '192.168.2.132' # 192.168.2.132


SERVER_PORT2 = 8002
s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s2.bind((SERVER_HOST, SERVER_PORT2))
s2.listen()
s2.settimeout(1000)


while 1:
    try:
        client_connection2, client_address2 = s2.accept()
        data = client_connection2.recv(BUFFER_SIZE).decode().strip()
        print("received data from mouse 2:", data)
        if data == "Begin":
            client_connection2.send(string_traversal_path.encode())
            time.sleep(.1)
            client_connection2.close()

    except socket.error as e2:
        data = ""
        print(str(SERVER_PORT2) + " " + str(e2))







