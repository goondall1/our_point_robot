import random



class CircleObsConfiguration:
    
    def __init__(self, width=1024,height=1024,radius_min=10,radius_max=120):
        self.radius_min = radius_min
        self.radius_max = radius_max
        self.width = width
        self.height = height

    def getConfiguration(self, cfg_num):
        return [(random.randint(self.radius_min,self.radius_max) ,random.randint(0,self.width),random.randint(0,self.height)) for _ in range(cfg_num)]



