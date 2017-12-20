# Server.py
import socket
import sys
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '127.0.0.1'
PORT = 8888
f_ack = b"f_ack\n"
s_ack = b"s_ack\n"
b_ack = b"b_ack\n"
l_ack = b"l_ack\n"
l_ok = b"l_ok\n"
r_ack = b"r_ack\n"
r_ok = b"r_ok\n"
D_ack = b"D_ack\n"


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
            if "f\n" in message.decode():
                sc.sendall(f_ack)
                print("replyed: ", f_ack)
            elif "s\n" in message.decode():
                sc.sendall(s_ack)
                print("replyed: ", s_ack)
            elif "b\n" in  message.decode():
                sc.sendall(b_ack)
                print("replyed: ", b_ack)
            elif "l" in message.decode():
                sc.sendall(l_ack)
                print("replyed: ", l_ack)
                time.sleep(2)
                sc.sendall(l_ok)
                print("replyed: ", l_ok)
            elif "r" in message.decode():
                sc.sendall(r_ack)
                print("replyed: ", r_ack)
                time.sleep(2)
                sc.sendall(r_ok)
                print("replyed: ", r_ok)
            elif "D" in message.decode():
                sc.sendall(D_ack)
                print("replyed: ", D_ack)
                print("press Enter to reply D_ok\\n")
                input()
                sc.sendall(b"D_ok")
            else:
                print("wrong format")
                sc.sendall(r_ack)
                print("replyed: ", r_ack)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print("*** connection failed.\n Delete the couple ***\n ERROR:")
            print(exc_type, exc_obj, exc_tb.tb_lineno)
            sc.close()
            del sc
            break
