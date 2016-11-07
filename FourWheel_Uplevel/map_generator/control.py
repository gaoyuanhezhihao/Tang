# control.py
from __future__ import print_function
import logging
from logging import handlers
import os

from view import MapView
from car_proxy import CarProxy
import data


MAP_STD_DIST = 100  # 100 pulse for a unit dist in map.


class MapControl():

    def __init__(self):
        self.init_logger()
        # init car.
        self.car = CarProxy(self.logger)
        # init view module.
        self.view = MapView(self.logger, self)
        # add data base.
        self.data_base = data.MapDataBase(self.logger)

        self.waiting_data = False

    def go_forward(self):
        if self.car.state in ['s', 'l', 'r']:
            self.data_base.record_order('F')
            self.car.map_forward()
        else:
            self.logger.warning("go forward only before you stop car")

    def save_path(self):
        self.data_base.save_path()

    def step_left(self, degree):
        self.car.step_left(degree)
        self.data_base.record_order('l', degree)

    def step_right(self, degree):
        self.car.step_right(degree)
        self.data_base.record_order('r', degree)

    def go_backward(self):
        self.data_base.record_order('B')
        self.car.map_backward()

    def stop(self):
        last_dist = self.car.map_stop()
        self.data_base.record_order('S', last_dist)

    def new_route(self, start_name, end_name):
        self.car.Stop()
        self.data_base.init_new_route(start=start_name, dst=end_name)

    def init_logger(self):
        _LOG_FORMAT = '%(asctime)s (%(filename)s/%(funcName)s)' \
            ' %(name)s %(levelname)s - %(message)s'
        self.logger = logging.getLogger('map_generator')
        _handler = logging.handlers.RotatingFileHandler(
            "./log/" + os.path.basename(__file__)[: -3] + ".log",
            maxBytes=102400, backupCount=20)
        _formatter = logging.Formatter(_LOG_FORMAT)
        _handler.setFormatter(_formatter)
        self.logger.addHandler(_handler)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(_formatter)
        self.logger.addHandler(ch)

    def loop(self):
        while True:
            self.view.update_panel()
            self.car.routines()

if __name__ == '__main__':
    cntr = MapControl()
    cntr.loop()
