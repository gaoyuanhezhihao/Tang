#! /usr/bin/python2
from __future__ import print_function
import socket
import time
import logging
import os
from platform import platform
from logging import handlers
import serial
from const_var import const
import threading

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '127.0.0.1'
PORT = 8888
const.split_flag = b'\r\n'
const.turn_over_time = 4
const.log_name = "mock_car"
# f_ack = b"f_ack"
# s_ack = b"s_ack"
# b_ack = b"b_ack"
# l_ack = b"l_ack"
# l_ok = b"l_ok"
# r_ack = b"r_ack"
# r_ok = b"r_ok"
# p_ack = b"p_ack"
# D_ack = b"D_ack"
# D_ok = b"D_ok"
# B_ack = b"B_ack"
# B_ok = b"B_ok"
# F_ack = b"F_ack"
# y_ack = b"y_ack"
# y_ok = b"y_ok"
# z_ack = b"z_ack"
# z_ok = b"z_ok"
# S_ack = b"Sab"
# Q_ack = b"Q_ack"
# K_ack = b"K_ack"


if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyUSB'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'


class Cmd(object):

    def __init__(self, success_reply):
        self.ack = ack
        self.complete_reply = success_reply

valid_cmd = ['s', 'p', 'l', 'r', 'f', 'b', 'F', 'B', 'x', 'y', 'I', 'K']
delay_reply_cmd = ['F', 'B', 'x', 'y', 'I', 'K']

class MockCar(object):

    def __init__(self):
        self.init_serial()
        self.logger.info("-"*40 + "new instance of mock car" + '-'*40)
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

    def call_back(self):
        if self.state is not 's':
            self.send(self.call_back_data)
            self.state = 's'

    def process_msg(self, message):
            print("recv", repr(message), "\n")
            msg_order = message[1].decode()
            if msg_order in valid_cmd:
                self.send(msg_order+'_ack')
                if msg_order in delay_reply_cmd:
                    self.call_back_data = msg_order + '_ok'
                    self.timer = threading.Timer(const.turn_over_time,
                                               self.call_back)
                    self.timer.start()
                elif msg_order == 'p':
                    print("change speed:", message[1], message[2])
            else:
                self.logger.error("wrong format %s", msg_order)

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
