# data.py

import logging
import json
import os
from const_var import const

const.map_folder_name = "map_data/"

class MapDataBase(object):

    def __init__(self, logger):
        self.logger = logger
        self.record_func_dict = {'S': self.record_S_order,
                                 'F': self.record_fb_order,
                                 'B': self.record_fb_order,
                                 'l': self.record_lr_order,
                                 'r': self.record_lr_order,
                                 }
        self.map_list = []
        self.last_order = 's'
        self.all_map = {}
        # create map data folder.
        if not os.path.exists(const.map_folder_name):
            os.makedirs(const.map_folder_name)

    def save_path(self):
        path_name = const.map_folder_name + self.start + \
                '-->' + self.dst + '.json'
        print("path_name:%s" % path_name)
        with open(path_name, 'w') as f:
            json.dump(self.map_list, f)
            f.close()
        print(self.map_list)

    def init_new_route(self, start, dst):
        self.logger.info("init new route")
        self.map_list = []
        self.last_order = 's'
        self.start = start
        self.dst = dst

    # def record_angle(self, order, angle):
        # if self.last_order in ['y', 'z']:
        # self.map_list.append((self.last_order, angle))
        # self.last_order = 'A'
        # else:
        # self.logger.debug("order before is not 'y' or 'z'")

    def record_order(self, order, data=None):
        self.logger.info("order: %s, data %s" % (order, data))
        func = self.record_func_dict[order]
        func(order, data)

    def record_lr_order(self, order, data):
        self.map_list.append((order, data))

    def record_fb_order(self, order, data):
        if self.last_order == order:
            pass
        elif self.last_order in ['S', 's']:
            self.last_order = order

    def record_S_order(self, order, dist):
        if self.last_order in ['F', 'B']:
            self.map_list.append((self.last_order, dist))
            self.last_order = 'S'
        else:
            self.logger.debug("order before 'S' is not 'F' or 'B'")
