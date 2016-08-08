# client_test_huanglu.py
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
                print "waiting turning complete msg..."
                rcv = s.recv(1024)
                if rcv == side + "_ok\n":
                    print "turning succeed"
                else:
                    print "turning FAILED, recv:", rcv
                continue
            elif msg == 'p':
                order = 'p'
                pwm = raw_input("pwm your want: ")
                s.sendall(order + '\n' + pwm + '\n')
            elif msg == 'D':
            	s.sendall("d\n")
            	s.sendall("D\n500")
            elif msg in ['f', 'b', 's']:
                s.sendall(msg+'\n')
            else:
                print "wrong order, ignored"
                continue

            print "recv:", s.recv(1024)
    except Exception, e:
        print "*** connection failed. \n", e, "\nRetrying"
