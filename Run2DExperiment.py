
import numpy as np
import sys
import getopt
from CircleObsConfiguration import CircleObsConfiguration as ObsCfg
from Circle import Circle
from PointRobotWithObsAndControl import PointRobotWithObsAndControl
from Plot2DBackground import PlotBackgroundWithObs
import Config


if __name__ == "__main__":
    height = Config.height
    width = Config.width
    num_of_start_goal_pairs = Config.num_of_start_goal_pairs

    try:
        opts, args = getopt.getopt(sys.argv[1:], "c:",["configuration"])
    except getopt.GetoptError:
        print("illegal configuration, input can only be a number from 1 to 10\n try: 'this_file.py' -c 1")
        sys.exit(2)
    
    numOfCircles = 1
    for opt, arg in opts:
        if opt in ("-c", "--configuration"):
            numOfCircles = int(arg)
    ppm_filename = './point_robot_with_obs.ppm'

    obs = [Circle(*obs) for obs in ObsCfg.getConfiguration(numOfCircles)] 

    preprocess = PlotBackgroundWithObs(obs,ppm_filename,background_size=(width,height))
    preprocess.prepare()

    start_goals = np.random.randint(0,height,size=(num_of_start_goal_pairs,4))

    num_of_solutions = 0
    num_of_failed_solutions = 0

    for row in start_goals:
        start = (int(row[0]),int(row[1]))        
        goal = (int(row[2]),int(row[3]))
        env = PointRobotWithObsAndControl(ppm_filename, obs, start, goal)
        if env.plan():
            env.recordSolution()
            env.save(ppm_filename)
            solution = env.getSolution()
            num_of_solutions += 1
        else:
            num_of_failed_solutions +=1
    print("Found {} valid solutions".format(num_of_solutions))
    print("Faild in {} solutions".format(num_of_failed_solutions))
    preprocess.plot()

