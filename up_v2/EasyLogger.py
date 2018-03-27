import logging
from logging.handlers import RotatingFileHandler
from logging import handlers
import sys
import os
import inspect

def track_prev_func():
    curFrame = inspect.currentframe()
    # print inspect.getouterframes(curFrame, 2)
    prev_frames = inspect.getouterframes(curFrame, 2)
    return prev_frames[2][1], prev_frames[2][3]

def get_log_level(s):
    log_level = s.upper()
    if log_level in "DEBUG":
        log_level = logging.DEBUG
    elif log_level in "INFO":
        log_level = logging.INFO
    elif log_level in "WARNING":
        log_level = logging.WARNING
    elif log_level in "ERROR":
        log_level = logging.ERROR
    elif log_level in "CRITICAL":
        log_level = logging.CRITICAL
    else:
        raise ValueError("no such level of logging --> %s" % log_level)
    return log_level

class EasyLogger(object):
    def __init__(self, name, f_level='debug', cmd_level='warning', log_dir = './log/'):
        self.log_list = []
        os.path.exists(log_dir) or os.makedirs(log_dir)
        f_log = logging.getLogger(name+'_f')
        f_log.setLevel(get_log_level(f_level))
        format = logging.Formatter('%(asctime)s %(levelname)s - %(message)s')
        fh = handlers.RotatingFileHandler(log_dir+name+'.log',
                                          maxBytes=(102400*5), backupCount=7)
        fh.setFormatter(format)
        f_log.addHandler(fh)
        self.log_list.append(f_log)

        cmd_log = logging.getLogger(name+'_cmd')
        cmd_log.setLevel(get_log_level(cmd_level))
        ch = logging.StreamHandler(sys.stdout)
        ch.setFormatter(format)
        cmd_log.addHandler(ch)
        self.log_list.append(cmd_log)

    def debug(self, info):
        call_fname, call_func = track_prev_func()
        new_info=" %s/%s--%s"%(os.path.split(call_fname)[-1], call_func, info)
        for lg in self.log_list:
            lg.debug(new_info)

    def info(self, info):
        call_fname, call_func = track_prev_func()
        new_info=" %s/%s--%s"%(os.path.split(call_fname)[-1], call_func, info)
        for lg in self.log_list:
            lg.info(new_info)

    def critical(self, info):
        call_fname, call_func = track_prev_func()
        new_info=" %s/%s--%s"%(os.path.split(call_fname)[-1], call_func, info)
        for lg in self.log_list:
            lg.critical(new_info)

    def warn(self, info):
        self.warning(info)

    def warning(self, info):
        call_fname, call_func = track_prev_func()
        new_info=" %s/%s--%s"%(os.path.split(call_fname)[-1], call_func, info)
        for lg in self.log_list:
            lg.warning(new_info)

    def error(self, info):
        call_fname, call_func = track_prev_func()
        new_info=" %s/%s--%s"%(os.path.split(call_fname), call_func, info)
        for lg in self.log_list:
            lg.error(new_info)







def init_logger(log_name, log_level, log_dir='./log/'):
    assert type(log_name) is str
    assert type(log_level) is str
    assert type(log_dir) is str

    log_level = log_level.upper()
    if log_level in "DEBUG":
        log_level = logging.DEBUG
    elif log_level in "INFO":
        log_level = logging.INFO
    elif log_level in "WARNING":
        log_level = logging.WARNING
    elif log_level in "ERROR":
        log_level = logging.ERROR
    elif log_level in "CRITICAL":
        log_level = logging.CRITICAL
    else:
        raise ValueError("no such level of logging --> %s" % log_level)
    os.makedirs(log_dir, exist_ok=True)
    log = logging.getLogger(log_name)
    log.setLevel(log_level)
    format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(format)
    log.addHandler(ch)

    fh = handlers.RotatingFileHandler(log_dir+log_name+'.log',
                                      maxBytes=(102400*5), backupCount=7)
    fh.setFormatter(format)
    log.addHandler(fh)
    return log
