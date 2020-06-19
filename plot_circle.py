import array
import matplotlib.pyplot as plt
import math


# PPM header
width = 1024
height = 1024
maxval = 255
ppm_header = f'P6 {width} {height} {maxval}\n'
image = array.array('B', [0, 0, 0] * width * height)

circle_center_x = math.floor(width/2)
circle_center_y = math.floor(height/2)
circle_radius = 50


def calcDistance(y, x):
    return math.sqrt(((x - circle_center_x)**2)+((y - circle_center_y)**2))


def inCirclePeri(y, x):
    dist = calcDistance(y, x)
    if(dist < circle_radius):
        return True
    return False


for y in range(circle_center_y-circle_radius, circle_center_y+circle_radius):
    for x in range(circle_center_x-circle_radius, circle_center_x+circle_radius):
        if(inCirclePeri(y, x)):
            index = 3 * (y * width + x)
            for i in range(3):
                image[index+i] = 255

with open('example.ppm', 'wb') as f:
    f.write(bytearray(ppm_header, 'ascii'))
    image.tofile(f)

img = plt.imread('example.ppm')
plt.imshow(img)
plt.show()
