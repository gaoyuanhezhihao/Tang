# Server.py
from __future__ import print_function
import socket
import time
import logging
import os
from platform import platform
from logging import handlers
import serial
from const_var import const

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '127.0.0.1'
PORT = 8888
const.split_flag = b'\r\n'
const.log_name = "mock_car"
f_ack = b"f_ack"
s_ack = b"s_ack"
b_ack = b"b_ack"
l_ack = b"l_ack"
l_ok = b"l_ok"
r_ack = b"r_ack"
r_ok = b"r_ok"
p_ack = b"p_ack"
D_ack = b"D_ack"
D_ok = b"D_ok"
B_ack = b"B_ack"
F_ack = b"F_ack"
y_ack = b"y_ack"
y_ok = b"y_ok"
z_ack = b"z_ack"
z_ok = b"z_ok"
S_ack = b"Sab"
Q_ack = b"Q_ack"


if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyS'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'


class MockCar(object):

    def __init__(self):
        self.init_serial()
        self.init_log()
        self.state = 's'

    def init_serial(self):
        print("serial port available:\n")
        print(self.serial_ports())
        sPortChoic = raw_input("Input the port number to open\n")
        self.port = serial.Serial(PORT_PREFIX+sPortChoic, 9600)

    def init_log(self):
        _LOG_FORMAT = '%(asctime)s (%(filename)s/%(funcName)s)' \
            ' %(name)s %(levelname)s - %(message)s'
        self.logger = logging.getLogger(const.log_name)
        _handler = logging.handlers.RotatingFileHandler(
            "./log/" +
            os.path.basename(__file__)[
                :-
                3] +
            ".log",
            maxBytes=102400,
            backupCount=20)
        _formatter = logging.Formatter(_LOG_FORMAT)
        _handler.setFormatter(_formatter)
        self.logger.addHandler(_handler)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(_formatter)
        self.logger.addHandler(ch)

    def serial_ports(self):
        ports = [PORT_PREFIX + str(i) for i in range(256)]
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def send(self, message):
        self.port.write(message+const.split_flag)
        self.logger.info("port sent:%s" % message)

    def process_msg(self, message):
            print("recv", repr(message), "\n")
            msg_order = message[1]
            if "f" == msg_order.decode():
                self.state = 'f'
                self.send(f_ack)
            elif "s" == msg_order.decode():
                self.state = 's'
                self.send(s_ack)
            elif "b" == msg_order.decode():
                self.state = 'b'
                self.send(b_ack)
            elif "l" == msg_order.decode():
                self.state = 'l'
                self.send(l_ack)
                time.sleep(2)
                self.send(l_ok)
            elif "r" == msg_order.decode():
                self.state = 'r'
                self.send(r_ack)
                time.sleep(2)
                self.send(r_ok)
            elif "D" == msg_order.decode():
                self.state = 'f'
                self.send(D_ack)
                self.send(D_ok)
            elif "p" == msg_order.decode():
                self.send(p_ack)
            elif "F" == msg_order.decode():
                self.state = 'f'
                self.send(F_ack)
            elif "B" == msg_order.decode():
                self.state = 'b'
                self.send(B_ack)
            elif "S" == msg_order.decode():
                self.state = 's'
                self.send(S_ack)
            elif "Q" == msg_order.decode():
                self.send(Q_ack + self.state)
            else:
                self.logger.error("wrong format")

    def loop(self):
        while True:
            bytes_wait = self.port.inWaiting()
            if bytes_wait >= 4:
                self.msg_rcv = self.port.read(bytes_wait)
                if self.msg_rcv[0] == 'H':
                    self.process_msg(self.msg_rcv)

if __name__ == '__main__':
    mock_car = MockCar()
    mock_car.loop()
