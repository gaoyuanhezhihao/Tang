# client_test_huanglu.py
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

def run(logger):
    SERVERIP = "127.0.0.1"
    SERVERPORT = 8888
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SERVERIP, SERVERPORT))
    for i in xrange(10):
        test_f(logger, s)
        time.sleep(2)
        test_s(logger, s)
        test_b(logger, s)
        time.sleep(2)
        test_s(logger, s)
        test_l(logger, s)
        test_s(logger, s)
        test_r(logger, s)
        test_s(logger, s)
        test_D(logger, s)
        test_s(logger, s)
        test_B(logger, s)
        test_s(logger, s)

def test_f(logger, sc):
    print("test_f")
    msg = "f\n"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)


def test_s(logger, sc):
    print("test_s")
    msg = "s\n"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)


def test_b(logger, sc):
    print("test_b")
    msg = "b\n"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)


def test_l(logger, sc):
    print("test_l")
    msg = "l\n90"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)
    if reply != "l_ok\n":
        logger.error("turn left :Wrong Reply")


def test_r(logger, sc):
    print("test_r")
    msg = "r\n90"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)
    if reply != "r_ok\n":
        logger.error("turn right:Wrong Reply")


def test_D(logger, sc):
    print("test_D")
    msg = "D\n80"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)
    if reply != "D_ok\n":
        logger.error("go dist forward:Wrong Reply")


def test_B(logger, sc):
    print("test_D")
    msg = "B\n80"
    sc.sendall(msg)
    logger.info("socket<==%s"%msg)
    reply = sc.recv(1024)
    logger.info("socket -->%s"%reply)
    if reply != "B_ok\n":
        logger.error("go dist backward:Wrong Reply")
if __name__ == '__main__':
    # create logger with 'spam_application'
    _LOG_FORMAT = '%(asctime)s- %(module)s- %(funcName)s-%(lineno)d-%(levelname)s-%(message)s'
    logger = logging.getLogger('auto_test')
    _handler = handlers.RotatingFileHandler("./log_auto_test/" +
            os.path.basename(__file__)[:-3] + ".log", maxBytes=102400, backupCount=50)
    _formatter = logging.Formatter(_LOG_FORMAT)
    _handler.setFormatter(_formatter)
    logger.addHandler(_handler)
    logger.setLevel(logging.INFO)
    run(logger)
