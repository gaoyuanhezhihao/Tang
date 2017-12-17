#!/usr/bin/python2
# car_proxy.py

from __future__ import print_function
from platform import platform
import logging
import serial
import struct
import time

from const_var import const

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
# const.peek_interval = 0.5  #peek state every 500ms
const.STD_PWM = 3200


class Waiting_Reply():

    def __init__(self, order_pack, reply_waiting,
                 deadline, success_callback,
                 overtime_Callback, wrong_callback):
        self.order_pack = order_pack
        self.reply_waiting = reply_waiting
        self.deadline = deadline
        self.success_callback = success_callback
        self.overtime_Callback = overtime_Callback
        self.wrong_callback = wrong_callback

    def __str__(self):
        return "{reply waiting:%s\ndeadline:%f\n}" %\
            (self.reply_waiting, self.deadline)

class CarProxy():

    def __init__(self, logger):
        self.logger = logger
        # init serial port
        print("serial port available:\n")
        print(self.serial_ports())
        sPortChoic = raw_input("Input the port number to open\n")
        self.port = serial.Serial(PORT_PREFIX+sPortChoic, 9600)
        # self.waiting_turn_ok = ''
        self.tmp_msg = ''
        self.state = 's'
        self.start_turn_time = time.time()
        self.msg_list = []
        self.last_peek_time = 0

        self.ack_waiting = None
        self.ok_waiting = None

    def success_ack(self, reply):
        order_pack = self.ack_waiting.order_pack
        order = self.ack_waiting.order_pack[0]
        if order in ['x', 'y', 'F', 'B', 'I', 'K']:
            self.ok_waiting = Waiting_Reply(order_pack, order+'_ok',
                                           time.time()+const.ok_time_lmt,
                                           self.success_ok,
                                            self.overtime_ok,
                                            self.wrong_ok)
        self.ack_waiting = None

    def overtime_ack(self):
        raise Exception("Car Error", "ACK package overtime")

    def wrong_ack(self):
        raise Exception("Car Error", "ACK package wrong")

    def success_ok(self):
        self.ok_waiting = None

    def overtime_ok(self):

    def wrong_ok(self):
        raise Exception("Car Error", "OK package wrong")

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

    def retry_last_order(self):
        self.send_verify_order(order=self.last_order, data1=self.last_data1,
                               data2=self.last_data2)

    def send_verify_order(self, order=None, data1=None, data2=None):
        self.__Send_Direct_Order(order, data1, data2)
        self.ack_waiting = Waiting_Reply((order, data1, data2),
                                         order+'_ack',
                                         time.time()+\
                                         const.ack_time_lmt,
                                         self.success_ack,
                                         self.overtime_ack,
                                         self.wrong_ack)
        if order in ['x', 'y', 'F', 'B', 'I', 'K']:
            self.ok_waiting = Waiting_Reply((order, data1, data2),
                                           order+'_ok',)

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
        self.send_verify_order(order='l')
        self.state = 'l'

    def turn_left_degree(self, degree):
        steps = int(degree * const.pulses_per_degree)
        self.step_left(steps)

    def step_left(self, steps):
        self.send_verify_order(order='x', data1=steps/256,
                               data2=steps % 256)
        self.start_turn_time = time.time()
        self.state = 'x'

    def turn_right(self):
        self.send_verify_order(order='r')
        self.state = 'r'

    def turn_right_degree(self, degree):
        steps = int(degree * const.pulses_per_degree)
        self.step_right(steps)

    def step_right(self, steps):
        self.send_verify_order(order='y', data1=steps/256,
                               data2=steps % 256)
        self.start_turn_time = time.time()
        self.state = 'y'

    def Forward(self):
        self.send_verify_order(order='f')
        self.state = 'f'

    def Backward(self):
        self.send_verify_order(order='b')
        self.state = 'b'

    def forward_dist(self, dist_in_cm):
        self.send_verify_order(order='F', data1=dist_in_cm/256,
                               data2=dist_in_cm % 256)
        self.start_go_dist_time = time.time()
        self.state = 'F'

    def backward_dist(self, dist_in_cm):
        self.send_verify_order(order='B', data1=dist_in_cm/256,
                               data2=dist_in_cm % 256)
        self.start_go_dist_time = time.time()
        self.state = 'B'

    def forward_steps(self, steps):
        self.send_verify_order(order='I', data1=steps/256,
                               data2=steps % 256)
        self.start_go_dist_time = time.time()
        self.state = 'I'

    def backward_steps(self, steps):
        self.send_verify_order(order='K', data1=steps/256,
                               data2=steps % 256)
        self.start_go_dist_time = time.time()
        self.state = 'K'

    def Stop(self):
        self.send_verify_order(order='s')
        self.state = 's'

    def __verify_or_retry(self, ack_msg):
        retry_count = 0
        while retry_count <= const.retry_limit:
            retry_count += 1
            flag, msg = self.rcv_ack(len(ack_msg))
            if ack_msg in msg:
                return True
        # connection failed.
        raise Exception("Car Error", "RETRY TOO TIMES")

    def rcv_ack(self, msg_len):
        msg_len += len(const.split_flag)
        start_time = time.time()
        bytes_waiting = 0
        while bytes_waiting < msg_len:
            if time.time() - start_time > const.ack_time_lmt:
                return False, "wait too long"
            bytes_waiting = self.port.inWaiting()
        rcv = self.port.read(bytes_waiting)
        return True, rcv

    def check_ok_msg(self):
        if self.state in ['x', 'y', 'F', 'B', 'I', 'K']:
            '''car is in turning state.'''
            ok_reply = '%s_ok' % self.state
            for idx, pack in enumerate(self.msg_list):
                if ok_reply in pack:
                    # got right reply.
                    self.state = 's'
                    del self.msg_list[0: idx+1]
                    self.logger.info("get reply:" + repr(pack))
                    return True
            # check if waits too long?
            if time.time() - self.start_turn_time > const.wait_turn_limit:
                self.peek_state()

    def __peek_state(self):
        def __success_peek(reply):
            reply[:5] == 'Q_ack'
            self.state = reply[5]

        def __wrong_peek(reply):
            self.logger.warning("peeked wrong reply->:%s"%reply)

        def __overtime_peek(reply):
            self.logger.warning("peek over time")

        self.__Send_Direct_Order(order='Q')
        assert self.ack_waiting is None
        self.ack_waiting = Waiting_Reply(('Q'), 'Q_ack',
                                         time.time()+const.ack_time_lmt,
                                         __success_peek, __wrong_peek,
                                         __overtime_peek)

    def peek_state(self):
        if time.time() - self.last_peek_time < const.peek_interval:
            # don't peek too frequent.
            return None
        else:
            self.last_peek_time = time.time()
        count_send = 0
        while count_send <= const.retry_limit:
            self.send_verify_order(order='Q')
            count_send += 1
            time_send_order = time.time()
            while time.time() - time_send_order < const.ack_time_lmt:
                self.rcv_uart_msg()
                for idx, pack in enumerate(self.msg_list):
                    if "Q_ack" == pack[0:5]:
                        self.state = pack[5]
                        del self.msg_list[0: idx+1]
                        self.logger.info("updated state to %s" %
                                         repr(self.state))
                        return True
        raise Exception("Car Error", "RETRY TOO TIMES")

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

    def rcv_uart_msg(self):
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.tmp_msg += rcv
            new_msg_list = self.tmp_msg.split(const.split_flag)
            if len(new_msg_list) > 1:
                self.tmp_msg = new_msg_list[-1]
                self.msg_list += new_msg_list[0:-1]
                return True
        return False
