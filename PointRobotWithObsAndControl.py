from ompl import util as ou
from ompl import base as ob
from ompl import control as oc
from Circle import *
from functools import partial
from Interfaces import State
import Config

class State2D(State):
    def __init__(self, state):
        self.x = state[0]
        self.y = state[1]

    def getCoordinates(self):
        return self.x, self.y
    

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
        self.bounds.setHigh(1024)

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
            # state_x, state_y = state[0],state[1]
            return spaceInformation.satisfiesBounds(state) and \
                    all([not circle.isInObs(State2D(state)) for circle in self.obs])

        def propagate(start, control, duration, state):
            for i in range(2):
                state[i] = start[i] + duration * control[i]

        self.setup.setStateValidityChecker(
            ob.StateValidityCheckerFn(partial(isStateValid, self.setup.getSpaceInformation())))
        self.setup.setStatePropagator(oc.StatePropagatorFn(propagate))

        tol = Config.distance_to_goal_tolerance
        self.setup.setStartAndGoalStates(start, goal, tol)
        si = self.setup.getSpaceInformation()

        # planner = oc.KPIECE1(si)
        planner = oc.RRT(si)
        self.setup.setPlanner(planner)

        step_size = Config.propagation_step_size
        si.setPropagationStepSize(step_size)
        solution_time = Config.solution_time
        return self.setup.solve(solution_time)

    def save(self, filename):
        self.ppm_.saveFile(filename)

    def getSolution(self):
        return self.setup.getSolutionPath()
    
    def getLastPathTime(self):
        return self.setup.getLastPlanComputationTime()

