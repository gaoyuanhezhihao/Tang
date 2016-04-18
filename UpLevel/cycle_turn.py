# cycle_turn.py
import time
import socket



SERVERIP = "127.0.0.1"
SERVERPORT = 8888
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((SERVERIP, SERVERPORT))
s.sendall("Boss\nBoss1\nConnectCarCar\nCar1")
raw_input("press any key to start")
while True:
    for i in range(4):
        msg = "l\n82\n"
        s.sendall(msg)
        print "recv:", s.recv(1024)
        print "waiting turning complete msg..."
        rcv = s.recv(1024)
        if rcv == side + "_ok\n":
            print "turning succeed"
        else:
            print "turning FAILED, recv:", rcv
        continue
    for j in range(4):
        msg = "r\n82\n"
        s.sendall(msg)
        print "recv:", s.recv(1024)
        print "waiting turning complete msg..."
        rcv = s.recv(1024)
        if rcv == side + "_ok\n":
            print "turning succeed"
        else:
            print "turning FAILED, recv:", rcv
        continue
