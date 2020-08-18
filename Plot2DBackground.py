

from typing import List
from Circle import *
import array
from PointRobotWithObsAndControl import State2D
import matplotlib.pyplot as plt

class PlotBackgroundWithObs:
    def __init__(self, obs: List[Circle], file_name = 'background.ppm', background_size = (1024,1024)):
        self.obs = obs 
        self.background_size = background_size
        self.file_name = file_name
        

    def prepare(self):
        # PPM header
        (width, height) = self.background_size
        maxval = 255
        ppm_header = f'P6 {width} {height} {maxval}\n'
        image = array.array('B', [0, 0, 0] * width * height)

        for circle in self.obs:
            for y in range(circle.y-circle.radius, circle.y+circle.radius):
                for x in range(circle.x-circle.radius, circle.x+circle.radius):
                    if(circle.isInObs(State2D((x, y)))):
                        index = 3 * (y * width + x)
                        if(index < 3*height*width):#draw only if index is in range of array
                            for i in range(3):
                                image[index+i] = 255

        with open(self.file_name, 'wb') as f:
            f.write(bytearray(ppm_header, 'ascii'))
            image.tofile(f)

    def plot(self):
        img = plt.imread(self.file_name)
        plt.imshow(img)
        plt.axis('off')
        plt.show()






