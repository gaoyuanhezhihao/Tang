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
import pdb

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HOST = '127.0.0.1'
PORT = 8888
const.split_flag = b'\r\n'
const.turn_over_time = 4
const.log_name = "mock_car"
const.cm_per_sec = 10
const.step_per_sec = 50
const.freq = 5
const.interval = 1.0/const.freq

if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyUSB'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'

COM_CMD = PORT_PREFIX+'2'
COM_ODOM = PORT_PREFIX+'3'

valid_cmd = ['s', 'p', 'l', 'r', 'f', 'b', 'F', 'B', 'L', 'R', 'I', 'K', 'Q']
delay_reply_cmd = ['F', 'B', 'L', 'R', 'I', 'K']
valid_states = ['s', 'f', 'b', 'l', 'r', 'L', 'R', 'F', 'B', 'I', 'K']
moving_states = ['f', 'b', 'l', 'r', 'F', 'B', 'L', 'R', 'I', 'K']

class MockCar(object):

    last_rep_tm = 0
    def __init__(self):
        self.init_serial()
        self.init_log()
        self.logger.info("-"*10 + "new instance of mock car" + '-'*10)
        self.state = 's'
        self.cm = 0
        self.step = 0

    def init_serial(self):
        print("serial port available:\n")
        print(self.serial_ports())
        self.port = serial.Serial(COM_CMD, 9600)
        self.port_odom = serial.Serial(COM_ODOM, 9600)

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

    def send_order(self, message):
        self.send(message, self.port)

    def send(self, message, port):
        port.write(message+const.split_flag)
        self.logger.info("port sent:%s" % message)

    def call_back(self):
        if self.state is not 's':
            self.send_order(self.call_back_data)
            self.state = 's'

    def process_msg(self, message):
        print("recv", repr(message), "\n")
        msg_order = message[1].decode()
        if msg_order in valid_cmd:
            self.send_order(msg_order+'_ack')
            if msg_order in delay_reply_cmd:
                self.call_back_data = msg_order + '_ok'
                self.timer = threading.Timer(const.turn_over_time,
                                           self.call_back)
                self.timer.start()
            elif msg_order == 'p':
                print("change speed:", message[1], message[2])
            elif msg_order == 'Q':
                self.logger.info('query state')
                self.send_order('Q_ack'+self.state)
            elif 's' == msg_order:
                self.cm = 0
                self.step = 0

            if msg_order in valid_states:
                self.state = msg_order
        else:
            self.logger.error("wrong format %s", msg_order)

    def send_odom_data(self):
        if self.state in ['f', 'b', 'F', 'B', 'I', 'K']:
            self.cm += const.cm_per_sec * const.interval
            # self.cm = int(self.cm)
        elif self.state in ['l', 'r', 'L', 'R']:
            self.step += const.cm_per_sec * const.interval
            # self.step = int(self.step)
        self.send("cm:{}".format(self.cm), self.port_odom)
        self.send("step:{}".format(self.step), self.port_odom)

    def check_report(self):

        if self.state != 's' and time.time() - self.last_rep_tm > const.interval:
            # pdb.set_trace()
            self.last_rep_tm = time.time()
            self.send_odom_data()

    def loop(self):
        while True:
            bytes_wait = self.port.inWaiting()
            if bytes_wait >= 4:
                self.msg_rcv = self.port.read(bytes_wait)
                if self.msg_rcv[0] == 'H':
                    self.process_msg(self.msg_rcv)
            self.check_report()

if __name__ == '__main__':
    mock_car = MockCar()
    mock_car.loop()
