# SocketControl_MPU6050.py
'''
This car admin software is used for Tang's car.
The down side is the arduino uno board.
'''
from __future__ import print_function
import logging
from logging import handlers
import time
import socket
import serial
from Calibrate_huanglu import CarAdmin
from threading import Thread
from inspect import currentframe, getframeinfo
import os
import sys


PULSES_PER_DEGREE = 1640.0/90.0
PULSES_PER_cm = 55.0
PULSES_STD = 50
TURN_SPEED = 0.1
GoSPEED = 0.3
# Valid_replies = ["D_ok", "B_ok", "s_ack", "f_ack", "b_ack", "l_ok", "r_ok"]
Valid_replies = ["l_ok", "r_ok"]
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
        self.sock_client = None
        self.exit = False

    def serial_client(self):
        bytes_waiting = self.port.inWaiting()
        if bytes_waiting >= 1:
            rcv = self.port.read(bytes_waiting)
            self.tmp_msg  += rcv
            self.msg_list = self.tmp_msg.split('\r\n')
            if len(self.msg_list) > 1:
                self.tmp_msg = self.msg_list[-1]
                for m in self.msg_list[:-1]:
                    logger.info("com-->:" + repr(m))
                    if m[0:4] in ['l_ok', 'r_ok']:
                        print("new pwm: ", self.pwm_std * GoSPEED)
                        self.Send_Direct_Order(order='p', data1=int(self.pwm_std * GoSPEED/256), data2=int(self.pwm_std * GoSPEED % 256))
                    if m in Valid_replies:
                        logger.info("socket<--: "+repr(m+"\n"))
                        self.send_msg_up(m+'\n')

    def send_msg_up(self, msg):
        try:
            if self.sock_client is None:
                return
            self.sock_client.sendall(msg)
        except KeyboardInterrupt:
            raise
        except Exception, e:
            logger.error("*** connection failed:"+e.message+"...Delete the couple ***")
            # print "*** connection failed.", e, "\n Delete the couple ***"
            # self.sock_client.shutdown(socket.SHUT_RDWR)
            # self.sock_client.close()


    def SocketClient(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVERIP, SERVERPORT))
        s.listen(1)
        while True:
            if self.exit:
                return
            print('Listening at'+str(s.getsockname()))
            self.sock_client, sockname = s.accept()
            logger.info('Socket connects'+str(self.sock_client.getsockname())+ 'and'+str(self.sock_client.getpeername()))
            while True:
                if self.exit:
                    return
                try:
                    message = self.sock_client.recv(1024)
                    print(message)
                    if message == '':
                        self.sock_client.shutdown()
                    logger.info("socket==>"+repr(message))
                    # print "recv", repr(message), '\n'
                    command_tokens = message.split('\n')
                    self.sock_client.sendall("ok\n")
                    if command_tokens[0] in ['f', 's', 'b']:
                        self.Send_Direct_Order(order = command_tokens[0])
                    elif command_tokens[0] in ['D','B'] and len(command_tokens) >= 2:
                        dist_in_cm = int(command_tokens[1])
                        self.Send_Direct_Order(order =  command_tokens[0], data1=dist_in_cm/256, data2= dist_in_cm%256)
                    elif command_tokens[0] in ['l', 'r'] and len(command_tokens) >= 2:
                        degree = int(command_tokens[1])
                        pulses = int(degree * PULSES_PER_DEGREE)
                        self.Send_Direct_Order(order='p', data1=int(self.pwm_std * TURN_SPEED/256), data2=int(self.pwm_std * TURN_SPEED % 256))
                        self.Send_Direct_Order(order = command_tokens[0], data1=pulses/256, data2=pulses%256)

                    elif command_tokens[0] == 'p' and len(command_tokens) >= 2:
                        self.pwm =  self.pwm_std * float(command_tokens[1])
                        self.Send_Direct_Order(order='p', data1=self.pwm/256, data2=self.pwm % 256)
                except KeyboardInterrupt:
                    raise
                except Exception, e:
                    self.Send_Direct_Order(order = 's')
                    self.sock_client = None
                    logger.error("*** connection failed:"+e.message+"...Delete the couple ***")
                    break

    def Run(self):
        ThreadSocket = Thread(target=self.SocketClient, args=())
        # ThreadSerialRead = Thread(target=self.ReadTheSerial, args=())
        # ThreadSerialRead.start()
        ThreadSocket.start()
        self.pwm_std = 3200
        self.pwm = 1000
        self.Send_Direct_Order(order='p', data1=self.pwm/256, data2=self.pwm % 256)
        while True:
            try:
                self.serial_client()
            except KeyboardInterrupt:
                print("!!!Run:Exit")
                self.exit = True
                os.system("kill $PPID")
                raise
            except Exception, e:
                print("!!!Exit")
                self.exit = True
                ThreadSocket.join()
                raise
        print("exit run")
if __name__ == '__main__':
    SERVERIP = '127.0.0.1'
    SERVERPORT = 8888
    # create logger with 'spam_application'
    _LOG_FORMAT = '%(asctime)s- %(module)s- %(funcName)s-%(lineno)d-%(levelname)s-%(message)s'
    logger = logging.getLogger('adaptor')
    _handler = handlers.RotatingFileHandler("./log/" +
            os.path.basename(__file__)[:-3] + ".log", maxBytes=102400, backupCount=50)
    _formatter = logging.Formatter(_LOG_FORMAT)
    _handler.setFormatter(_formatter)
    logger.addHandler(_handler)
    logger.setLevel(logging.INFO)
    Admin = CarSocketAdmin('CarCar', SERVERIP, SERVERPORT, 1)
    Admin.Run()
