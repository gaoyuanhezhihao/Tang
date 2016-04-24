# Client_Indoor.py
import socket


SERVERIP = "127.0.0.1"
SERVERPORT = 8888
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        s.connect((SERVERIP, SERVERPORT))
        s.sendall("Boss\nBoss1\nConnectCarCar\nCar1")
        while True:
            msg = raw_input("Your command:")
            if msg in ['l', 'r']:
                side = msg
                angle = raw_input("angle:")
                msg +='\n'
                msg += angle
                s.sendall(msg+'\n')
                print "recv:", s.recv(1024)
                print "waiting turning complete msg..."
                rcv = s.recv(1024)
                if rcv == side + "_ok\n":
                    print "turning succeed"
                else:
                    print "turning FAILED, recv:", rcv
                continue
            elif msg in "mnxy":
                order = msg
                time_of_push = raw_input("time of the push: ")
                s.sendall(order + '\n' + time_of_push + '\n')
            elif msg == 'p':
                order = 'p'
                pwm = raw_input("pwm your want: ")
                s.sendall(order + '\n' + pwm + '\n')
            else:
                s.sendall(msg+'\n')
            print "recv:", s.recv(1024)
    except Exception, e:
        print "*** connection failed. \n", e, "\nRetrying"
