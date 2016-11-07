# control.py
'''
control module for map walker system.
'''

import logging
from logging import handlers
import os
import Queue
from functools import partial
from data import MapWalkerData
from view import MapWalkerView
from car_proxy import CarProxy

from const_var import const


class State(object):
    '''
    state of the map walk stage.
    '''
    def __init__(self, order, data, car, logger, routines):
        self.car = car
        self.logger = logger
        self.order = order
        self.data = data
        self.order_func = self.generate_pack(order)
        self.routines = routines

    def generate_pack(self, order):
        order2func = {'F': self.car.go_dist,
                      'K': self.car.go_backward,
                      'l': self.car.step_left,
                      'r': self.car.step_right}
        return order2func[order]

    def execute(self):
        self.order_func(self.data)
        while self.car.state is not 's':
            for routine in self.routines:
                routine()


class MapWalkerControl():

    def __init__(self):
        self.init_logger()
        # init car.
        self.car = CarProxy(self.logger)
        # init view module.
        self.view = MapWalkerView(self.logger, self)
        # add data base.
        self.data_base = MapWalkerData(self.logger)

    def walk_to(self, start, dst):
        '''
        walk from the start position to the destination.
        '''
        # request route.
        path = self.data_base.read_map(start, dst)
        # init state queue.
        states_queue = Queue.Queue()
        for node in path:
            states_queue.put(State(node[0], node[1],
                                   self.car, self.logger,
                                   [self.view.routines,
                                    self.car.routines]))

        # execute state one by one.
        while not states_queue.empty():
            state = states_queue.get()
            state.execute()

    def init_logger(self):
        _LOG_FORMAT = '%(asctime)s (%(filename)s/%(funcName)s)' \
            ' %(name)s %(levelname)s - %(message)s'
        self.logger = logging.getLogger('map_walker')
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
            self.view.routines()
            self.car.routines()

if __name__ == '__main__':
    cntr = MapWalkerControl()
    cntr.loop()
