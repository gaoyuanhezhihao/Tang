# Server.py
import socket
import sys
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '127.0.0.1'
PORT = 6790
ack = "ok\n"
l_ack = "l_ok\n"
r_ack = "r_ok\n"


def recv_all(sock, length):
    data = ""
    while len(data) < length:
        more = sock.recv(length - len(data))
        if not more:
            raise EOFError('recv_all')
        data += more
    return data


s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(1)
CaptainDict = {}
CarDict = {}
last_msg = " "
while True:
    print('Listening at', s.getsockname())
    sc, sockname = s.accept()
    print('We have accepted a connection from ', sockname)
    print('Socket connects', sc.getsockname(), 'and', sc.getpeername())
    while True:
        try:
            message = sc.recv(1024)
            print("recv", repr(message), "\n")
            if "f\n" == message.decode() or "b\n" == message.decode():
                sc.sendall(ack.encode())
                print("replyed: ", ack)
            elif "l" in message.decode():
                sc.sendall(ack.encode())
                print("replyed: ", ack)
                time.sleep(2)
                sc.sendall(l_ack.encode())
                print("replyed: ", l_ack)
            elif "r" in message.decode():
                sc.sendall(ack.encode())
                print("replyed: ", ack)
                time.sleep(2)
                sc.sendall(r_ack.encode())
                print("replyed: ", r_ack)
            else:
                print("wrong format")
                sc.sendall(ack.encode())
                print("replyed: ", ack)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print("*** connection failed.\n Delete the couple ***\n ERROR:")
            print(exc_type, exc_obj, exc_tb.tb_lineno)
            sc.close()
            del sc
            break
