import random

radius_min = 10
radius_max = 120
width = 1024
height = 1024
num_configurations = 10
cfg_dict = {
    i:[(random.randint(radius_min,radius_max) ,random.randint(0,width),random.randint(0,height)) for _ in range(i)] for i in range(1,num_configurations+1)
}

class Configuration:
    @staticmethod
    def getConfiguration(cfg_num):
        return cfg_dict[cfg_num]




