import math
from Interfaces import *


class Circle(Obstacle):
    def __init__(self, radius, x, y):
        self.radius = radius
        self.x = x 
        self.y = y

    def isInObs(self, state: State):
        (x , y) = state.getCoordinates()
        return math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2) <= self.radius
