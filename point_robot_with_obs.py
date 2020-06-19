from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
from functools import partial
import matplotlib.pyplot as plt
from ompl import util as ou
import math
import array
import numpy as np

width = 100
height = 100


# maxval = 255
# ppm_header = f'P6 {width} {height} {maxval}\n'
# #
# # # PPM image data (filled with blue)
# image = array.array('B', [0, 0, 0] * width * height)

# Fill with red the rectangle with origin at (10, 10) and width x height = 50 x 80 pixels
# for y in range(10, 90):
#     for x in range(10, 60):
#         index = 3 * (y * width + x)
#         image[index] = 0           # red channel
#         image[index + 1] = 0         # green channel
#         image[index + 2] = 0         # blue channel
#
# with open('white_red_example.ppm', 'wb') as f:
#     f.write(bytearray(ppm_header, 'ascii'))
#     image.tofile(f)
# #
# img = plt.imread("./white_red_example.ppm")
# plt.imshow(img)
# plt.axis('off')
# plt.show()

class PointRobotWithObsAndControl:
    # def __init__(self, ppm_file, obs):
    def __init__(self, ppm_file):
        self.ppm_ = ou.PPM()
        self.ppm_.loadFile(ppm_file)
        self.width = self.ppm_.getWidth()
        self.height = self.ppm_.getHeight()
        self.space = ob.RealVectorStateSpace()
        self.space.addDimension(0.0, self.width)
        self.space.addDimension(0.0, self.height)
        self.bounds = ob.RealVectorBounds(2)
        # bounds.addDimension(0.0, width)
        # bounds.addDimension(0.0, height)
        self.bounds.setLow(0)
        self.bounds.setHigh(1000)
        self.space.setBounds(self.bounds)
        self.cspace = oc.RealVectorControlSpace(self.space, 2)
        self.cbounds = ob.RealVectorBounds(2)
        self.cbounds.setLow(-1)
        self.cbounds.setHigh(1)
        self.cspace.setBounds(self.cbounds)
        self.setup = oc.SimpleSetup(self.cspace)

    def recordSolution(self):
        solution = self.setup.getSolutionPath()
        p = solution
        p.interpolate()
        for i in range(p.getStateCount()):
            w = min(self.width - 1, int(p.getState(i)[0]))
            h = min(self.height - 1, int(p.getState(i)[1]))
            c = self.ppm_.getPixel(h, w)
            c.red = 255
            c.green = 0
            c.blue = 0

    def plan(self, start_row=0, start_col=0, goal_row=55, goal_col=55):
        start = ob.State(self.space)
        start[0] = start_row
        start[1] = start_col
        goal = ob.State(self.space)
        goal[0] = goal_row
        goal[1] = goal_col

        #
        # print(start)
        # print(goal)
        #
        def isStateValid(spaceInformation, state):
            # perform collision checking or check if other constraints are satisfied
            radius = 50
            center_x = 512
            center_y = 512
            return spaceInformation.satisfiesBounds(state) and \
                   math.sqrt((state[0] - center_x) ** 2 + (state[1] - center_y) ** 2) > radius

        def propagate(start, control, duration, state):
            for i in range(2):
                # print("duration is: ",duration)
                state[i] = start[i] + duration * control[i]
            # for i, (s, c) in enumerate(zip(start, control)):
            #     state[i] = s + duration * c

        self.setup.setStateValidityChecker(
            ob.StateValidityCheckerFn(partial(isStateValid, self.setup.getSpaceInformation())))
        self.setup.setStatePropagator(oc.StatePropagatorFn(propagate))

        tol = 0.05
        self.setup.setStartAndGoalStates(start, goal, tol)
        si = self.setup.getSpaceInformation()

        # planner = oc.KPIECE1(si)
        planner = oc.RRT(si)
        self.setup.setPlanner(planner)

        si.setPropagationStepSize(0.5)
        return self.setup.solve(20.0)

    def save(self, filename):
        self.ppm_.saveFile(filename)

    def getSolution(self):
        return self.setup.getSolutionPath()


if __name__ == "__main__":

    env = PointRobotWithObsAndControl("./example.ppm")

    if env.plan(20, 20, 800, 800):
        env.recordSolution()
        env.save("result_demo.ppm")
        solution = env.getSolution()

        # controls = solution.getControls()
        # control0 = controls[0]
        # states = solution.getStates()

        # print the path to screen
        print("Found solution:\n%s" % solution.printAsMatrix())
        # a = np.array(solution.printAsMatrix())
        # print(a)
        env.save("./result_demo.ppm")
        img = plt.imread("./result_demo.ppm")
        plt.imshow(img)
        plt.axis('off')
        plt.show()
