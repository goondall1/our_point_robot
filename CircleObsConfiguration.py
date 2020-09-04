import random
import Config

class CircleObsConfiguration:

    @staticmethod
    def getConfiguration(cfg_num):
        radius_min = Config.radius_min
        radius_max = Config.radius_max
        width = Config.width
        height = Config.height
        return [
            (random.randint(radius_min,radius_max) ,
            random.randint(radius_max,width - radius_max),
            random.randint(radius_max,height - radius_max)) for _ in range(cfg_num)]

    @staticmethod
    def getBlockingCircles():
        radius_min = Config.radius_min
        radius_max = Config.radius_max
        width = Config.width
        height = Config.height
        return [
            (33 ,
            500,
            32+ i * 64) for i in range(16)]



