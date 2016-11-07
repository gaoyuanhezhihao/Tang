# data.py
'''
data module for map walker system.
'''

import json
from const_var import const

const.map_folder_name = "map_data/"


class MapWalkerData(object):

    def __init__(self, logger):
        self.logger = logger

    def read_map(self, start, dst):
        map_list = []
        file_name = const.map_folder_name + start + '-->' + dst + '.json'
        with open(file_name, 'r') as f:
            map_list = json.load(f)
            f.close()
        self.logger.info("readed map:%s" % str(map_list))
        return map_list
