from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
from functools import partial
import matplotlib.pyplot as plt
from ompl import util as ou
import math
import array
import numpy as np
import sys
from typing import List
import getopt
from CircleObsConfiguration import Configuration as Cfg


class Circle:
    def __init__(self, radius, x, y):
        self.radius = radius
        self.x = x 
        self.y = y
    
    def isInCircle(self, x ,y):
        return math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2) <= self.radius

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
                    if(circle.isInCircle(x, y)):
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


class PointRobotWithObsAndControl:
    # def __init__(self, ppm_file, obs):
    def __init__(self, ppm_file,obs : list, start=(1,1), goal=(1000,1000)):
        self.ppm_ = ou.PPM()
        self.ppm_.loadFile(ppm_file)

        self.width = self.ppm_.getWidth()
        self.height = self.ppm_.getHeight()

        self.space = ob.RealVectorStateSpace()
        self.space.addDimension(0.0, self.width)
        self.space.addDimension(0.0, self.height)

        self.bounds = ob.RealVectorBounds(2)
        self.bounds.setLow(0)
        self.bounds.setHigh(1000)

        self.space.setBounds(self.bounds)
        self.cspace = oc.RealVectorControlSpace(self.space, 2)
        self.cbounds = ob.RealVectorBounds(2)
        self.cbounds.setLow(-1)
        self.cbounds.setHigh(1)
        self.cspace.setBounds(self.cbounds)

        self.setup = oc.SimpleSetup(self.cspace)
        self.obs = obs
        self.start = start
        self.goal = goal
        

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

    def plan(self):

        start = ob.State(self.space)
        (start[0], start[1]) = self.start

        goal = ob.State(self.space)
        (goal[0], goal[1]) = self.goal
        
        def isStateValid(spaceInformation, state):
            # perform collision checking or check if other constraints are satisfied
            state_x, state_y = state[0],state[1]
            return spaceInformation.satisfiesBounds(state) and \
                    all([not circle.isInCircle(state_x, state_y) for circle in self.obs])

        def propagate(start, control, duration, state):
            for i in range(2):
                state[i] = start[i] + duration * control[i]

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

    try:
        opts, args = getopt.getopt(sys.argv[1:], "c:",["configuration"])
    except getopt.GetoptError:
        print("illegal configuration, input can only be a number from 1 to 10\n try: 'this_file.py' -c 1")
        sys.exit(2)
    
    cfg = 1
    for opt, arg in opts:
        if opt in ("-c", "--configuration"):
            cfg = int(arg)

    obs = [Circle(*obs) for obs in Cfg.getConfiguration(cfg)]
    ppm_filename = './bg_w_2_circles.ppm'
    preprocess = PlotBackgroundWithObs(obs,ppm_filename)
    preprocess.prepare()

    start_goals = np.random.randint(0,1024,size=(1000,4))
    num_of_solutions = 0
    num_of_failed_solutions = 0
    for row in start_goals:
        start = (int(row[0]),int(row[1]))        
        goal = (int(row[2]),int(row[3]))
        env = PointRobotWithObsAndControl(ppm_filename,obs,start,goal)
        if env.plan():
            env.recordSolution()
            env.save(ppm_filename)
            solution = env.getSolution()
            num_of_solutions += 1
            # print the path to screen
            # print("Found solution:\n%s" % solution.printAsMatrix())
        else:
            num_of_failed_solutions +=1
    print("Found {} valid solutions".format(num_of_solutions))
    print("Faild in {} solutions".format(num_of_failed_solutions))
    preprocess.plot()

