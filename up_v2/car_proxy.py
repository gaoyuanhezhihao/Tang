#!/usr/bin/python2
# car_proxy.py

from __future__ import print_function
from platform import platform
import logging
import serial
import struct
import time
from Queue import Queue
import pdb
from const_var import const
import os

if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyUSB'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'
const.retry_limit = 2  # if failed after 2 retries, Raise error.
const.ack_time_lmt = 1.0  # wait no more than 1000 ms before receiving reply.
const.ok_time_lmt = 2.0
const.pulses_per_degree = 1720/90
const.pulses_per_cm = 65.57
const.split_flag = '\r\n'
const.peek_interval = 2
# const.peek_interval = 0.5  #peek state every 500ms
const.STD_PWM = 3200

COM1 = PORT_PREFIX+'0'
COM2 = PORT_PREFIX+'1'

const.log_dir = './car_proxy.log/'


class CarProxy():
    __state__
    def __set_state__(self, new_state):
        self.__state__ = new_state

    def init_log(self):
        if not os.path.exists(const.log_dir):
            os.makedirs(const.log_dir)
        _LOG_FORMAT = '%(asctime)s (%(filename)s/%(funcName)s)' \
            ' %(name)s %(levelname)s - %(message)s'
        self.logger = logging.getLogger("car_proxy")
        _handler = logging.handlers.RotatingFileHandler(
            const.log_dir + os.path.basename(__file__)[:-3]+".log",
            maxBytes=102400, backupCount=20)
        _formatter = logging.Formatter(_LOG_FORMAT)
        _handler.setFormatter(_formatter)
        self.logger.addHandler(_handler)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(_formatter)
        self.logger.addHandler(ch)

    def __init__(self, logger=None):
        self.logger = logger
        if None is self.logger:
            self.init_log()
        # init serial port
        ports = self.serial_ports()
        if COM1 not in ports or COM2 not in ports:
            print("you need to reconfig the COMs !!!")
            print("serial port available:\n")
            print(ports)
            raise Exception("Init Fail", "com not exist")
        self.port = serial.Serial(COM1, 9600)
        self.odom_port = serial.Serial(COM2, 9600)
        # self.waiting_turn_ok = ''
        self.tmp_msg = ''
        self.tmp_odom_msg = ''
        self.__set_state__('s')
        self.start_turn_time = time.time()
        self.msg_list = []
        self.cm_que = Queue()
        self.step_que = Queue()
        self.last_peek_time = 0

        # self.rcv_callback = {r'._ok': self.__ok_receive__,
                             # r'state:.': self.__state_update__}

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

    def send_verify_order(self, order=None, data1=None, data2=None):
        self.__Send_Direct_Order(order, data1, data2)
        self.__verify_or_retry(order+'_ack')

    def __Send_Direct_Order(self, order=None, data1=None, data2=None):
        if data1 is None or data2 is None:
            data1 = 0x00
            data2 = 0x00
        # self.port.write('H')
        # self.port.write(order)
        # self.port.write(struct.pack('!B', data1))
        # self.port.write(struct.pack('!B', data2))
        msg = 'H'
        msg += order
        msg += struct.pack('!B', data1)
        msg += struct.pack('!B', data2)
        self.port.write(msg)
        self.last_order = order
        self.last_data1 = data1
        self.last_data2 = data2
        self.right_ack = self.last_order + '_ack'
        self.send_msg_time = time.time()
        self.logger.info("Send_Direct_Order:"+msg)

    def __set_pwm(self, new_pwm):
        self.pwm = int(new_pwm)
        self.send_verify_order(order='p', data1=self.pwm/256,
                               data2=self.pwm % 256)

    def change_speed(self, speed):
        new_pwm = const.STD_PWM * speed
        self.__set_pwm(new_pwm)

    def turn_left(self):
        if 'l' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='l')
        self.__set_state__('l')

    def turn_left_degree(self, degree):
        steps = int(degree * const.pulses_per_degree)
        self.step_left(steps)

    def step_left(self, steps):
        if 'L' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='L', data1=steps/256,
                               data2=steps % 256)
        self.start_turn_time = time.time()
        self.__set_state__('L')

    def turn_right(self):
        if 'r' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='r')
        self.__set_state__('r')

    def turn_right_degree(self, degree):
        steps = int(degree * const.pulses_per_degree)
        self.step_right(steps)

    def step_right(self, steps):
        if 'R' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='R', data1=steps/256,
                               data2=steps % 256)
        self.start_turn_time = time.time()
        self.__set_state__('R')

    def forward(self):
        if 'f' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='f')
        self.__set_state__('f')

    def backward(self):
        if 'b' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='b')
        self.__set_state__('b')

    def forward_dist(self, dist_in_cm):
        if 'F' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='F', data1=dist_in_cm/256,
                               data2=dist_in_cm % 256)
        self.start_go_dist_time = time.time()
        self.__set_state__('F')

    def backward_dist(self, dist_in_cm):
        if 'B' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='B', data1=dist_in_cm/256,
                               data2=dist_in_cm % 256)
        self.start_go_dist_time = time.time()
        self.__set_state__('B')

    def forward_step(self, steps):
        if 'I' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='I', data1=steps/256,
                               data2=steps % 256)
        self.start_go_dist_time = time.time()
        self.__set_state__('I')

    def backward_step(self, steps):
        if 'K' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='K', data1=steps/256,
                               data2=steps % 256)
        self.start_go_dist_time = time.time()
        self.__set_state__('K')

    def stop(self):
        if 's' == state:
            self.logger.warn("duplicate '%s' order, ignored"%state)
            return
        self.send_verify_order(order='s')
        self.__set_state__('s')

    def __verify_or_retry(self, ack_msg):
        self.rcv_uart_msg()
        while True:
            if ack_msg in self.msg_list:
                i = self.msg_list.index(ack_msg)
                self.msg_list = self.msg_list[i+1:]
                return True
            self.rcv_uart_msg()
        # connection failed.
        raise Exception("Car Error", "RETRY TOO TIMES")

    def check_ok_msg(self):
        if self.state in ['L', 'R', 'F', 'B', 'I', 'K']:
            '''car is in turning state.'''
            ok_reply = '%s_ok' % self.state
            for idx, pack in enumerate(self.msg_list):
                # pdb.set_trace()
                if ok_reply in pack:
                    # got right reply.
                    self.__set_state__('s')
                    del self.msg_list[0: idx+1]
                    self.logger.info("OK_MSG:" + repr(pack))
                    return True

    def get_car_stae(self):
        retry_count = 0
        good_reply = "Q_ackx"
        while retry_count <= const.retry_limit:
            retry_count += 1
            flag, msg = self.rcv_ack(len(good_reply))
            self.logger.info("rcv msg:%s", repr(msg))
            if flag and "Q_ack" in msg:
                return msg[5]
        # connection failed.
        raise Exception("Car Error", "RETRY TOO TIMES")

    def routines(self):
        self.rcv_uart_msg()
        self.check_ok_msg()
        self.read_odom()

    def rcv_uart_msg(self):
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.logger.info("rcv:'{}'".format(rcv))
            self.tmp_msg += rcv
            new_msg_list = self.tmp_msg.split(const.split_flag)
            if len(new_msg_list) > 1:
                self.tmp_msg = new_msg_list[-1]
                self.msg_list += new_msg_list[0:-1]
                self.logger.info("msg list add:{}".format(new_msg_list[0:-1]))
                return True
        return False

    def read_odom(self):
        bytes_waiting = self.odom_port.inWaiting()
        if bytes_waiting >= 1:
            rcv = self.odom_port.read(bytes_waiting)
            self.tmp_odom_msg += rcv
            new_msg_list = self.tmp_odom_msg.split(const.split_flag)

            self.tmp_odom_msg = new_msg_list[-1]
            for pack in new_msg_list[:-1]:
                if pack[:3] == 'cm:':
                    self.logger.info("cm que << {}".format(float(pack[3:])))
                    self.cm_que.put(float(pack[3:]))
                elif pack[:5] == 'step:':
                    self.logger.info("step que << {}".format(float(pack[5:])))
                    self.step_que.put(float(pack[5:]))
