# SocketControl_MPU6050.py
'''
This car admin software is used for Tang's car.
The down side is the arduino uno board.
'''
import logging
import time
import socket
import serial
from Calibrate import CarAdmin
from threading import Thread
import Queue


PULSES_PER_DEGREE = 1720/90
PULSES_PER_cm = 65.57
PULSES_STD = 50
Valid_replies = ["D_ok","y_ok", "z_ok", "y_ack", "z_ack" , "D_ack", "s_ack", "f_ack", "b_ack", "g_ack", "w_ack", "l_ack", "r_ack", "p_ack"]
def recv_all(sock, length):
    data = ""
    while len(data) < length:
        more = sock.recv(length - len(data))
        if not more:
                raise EOFError('recv_all')
        data += more
        return data


class CarSocketAdmin(CarAdmin):

    def __init__(self, name, ServerIP, ServerPort, ID):
        CarAdmin.__init__(self, name)
        self.ServerIP = ServerIP
        self.ServerPort = ServerPort
        self.down_queue = Queue.Queue(maxsize = 0) # infinite length.
        self.up_queue = Queue.Queue(maxsize = 0)
        self.sock_client = None

    def serial_client(self):
        bytes_waiting = self.port.inWaiting()
        if bytes_waiting >= 1:
            rcv = self.port.read(bytes_waiting)
            self.tmp_msg  += rcv
            self.msg_list = self.tmp_msg.split('\r\n')
            if len(self.msg_list) > 1:
                self.tmp_msg = self.msg_list[-1]
                for m in self.msg_list[:-1]:
                    logger.info("recv: " + repr(m))
                    if m in Valid_replies:
                        logger.info("send up: "+m+"\n")
                        self.send_msg_up(m+'\n')

    def send_msg_up(self, msg):
        try:
            if self.sock_client is None:
                return
            self.sock_client.sendall(msg)
        except Exception, e:
            logger.error("*** connection failed."+str(e)+"\n Delete the couple ***")
            # print "*** connection failed.", e, "\n Delete the couple ***"
            # self.sock_client.shutdown(socket.SHUT_RDWR)
            # self.sock_client.close()


    def SocketClient(self):
        time_to_push = 0
        LegalOrder = ['g', 'l', 'r', 'f', 's', 'b', 'p']
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVERIP, SERVERPORT))
        s.listen(1)
        while True:
            print 'Listening at'+str(s.getsockname())
            self.sock_client, sockname = s.accept()
            logger.info( 'We have accepted a connection from '+str(sockname))
            logger.info('Socket connects'+str(self.sock_client.getsockname())+ 'and'+str(self.sock_client.getpeername()))
            while True:
                try:
                    message = self.sock_client.recv(1024)
                    if message == '':
                        self.sock_client.shutdown()
                    logger.info("recv"+repr(message))
                    # print "recv", repr(message), '\n'
                    command_tokens = message.split('\n')
                    if command_tokens[0] in ['g', 'w', 'l', 'r', 'f', 's', 'b']:
                        self.Send_Direct_Order(order = command_tokens[0])
                    elif command_tokens[0] in ['D',] and len(command_tokens) >= 2:
                        dist_in_cm = int(command_tokens[1])
                        dist_in_pulse = int(PULSES_PER_cm * dist_in_cm)
                        self.Send_Direct_Order(order = 'd', data1=PULSES_STD/256, data2=PULSES_STD%256)
                        self.Send_Direct_Order(order = 'D', data1=dist_in_pulse/PULSES_STD/256, data2= dist_in_pulse/PULSES_STD%256)
                    elif command_tokens[0] in ['y', 'z'] and len(command_tokens) >= 2:
                        degree = int(command_tokens[1])
                        pulses = degree * PULSES_PER_DEGREE
                        self.Send_Direct_Order(order = command_tokens[0], data1=pulses/256, data2=pulses%256)

                    elif command_tokens[0] == 'p' and len(command_tokens) >= 2:
                        self.pwm =  self.pwm_std * float(command_tokens[1])
                        self.Send_Direct_Order(order='p', data1=self.pwm/256, data2=self.pwm % 256)
                except Exception, e:
                    self.Send_Direct_Order(order = 's')
                    self.sock_client = None
                    logger.error("*** connction failed."+str(e)+"\n Delete the couple ***")
                    break

    def Run(self):
        ThreadSocket = Thread(target=self.SocketClient, args=())
        # ThreadSerialRead = Thread(target=self.ReadTheSerial, args=())
        # ThreadSerialRead.start()
        ThreadSocket.start()
        self.pwm_std = 1600
        self.pwm = 1600
        self.Send_Direct_Order(order='p', data1=self.pwm/256, data2=self.pwm % 256)
        while True:
            self.serial_client()
if __name__ == '__main__':
    SERVERIP = '127.0.0.1'
    SERVERPORT = 8888
    # create logger with 'spam_application'
    logger = logging.getLogger('SocketControl_MPU6050')
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler('SocketControl_MPU6050.log')
    fh.setLevel(logging.DEBUG)
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.ERROR)
    # create console handler to debug. 
    debug_h = logging.StreamHandler()
    debug_h.setLevel(logging.INFO)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    debug_h.setFormatter(formatter)
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(debug_h)
    logger.addHandler(fh)
    logger.addHandler(ch)
    Admin = CarSocketAdmin('CarCar', SERVERIP, SERVERPORT, 1)
    Admin.Run()
