
import numpy as np
import sys
import getopt
import matplotlib.pyplot as plt
from CircleObsConfiguration import CircleObsConfiguration as ObsCfg
from Circle import Circle
from PointRobotWithObsAndControl import PointRobotWithObsAndControl
from Plot2DBackground import PlotBackgroundWithObs
import Config
import os


if __name__ == "__main__":
    height = Config.height
    width = Config.width
    num_of_start_goal_pairs = Config.num_of_start_goal_pairs

    # try:
    #     opts, args = getopt.getopt(sys.argv[1:], "c:",["configuration"])
    # except getopt.GetoptError:
    #     print("illegal configuration, input can only be a number from 1 to 10\n try: 'this_file.py' -c 1")
    #     sys.exit(2)
    
    # for opt, arg in opts:
    #     if opt in ("-c", "--configuration"):
    #         numOfCircles = int(arg)

    numOfCircles = Config.num_of_obs
    ppm_filename = './point_robot_with_obs.ppm'

    obs = [Circle(*obs) for obs in ObsCfg.getConfiguration(numOfCircles)] 
    # obs = [Circle(*obs) for obs in ObsCfg.getBlockingCircles()] 

    preprocess = PlotBackgroundWithObs(obs,ppm_filename,background_size=(width,height))
    preprocess.prepare()

    start_goals = np.random.randint(0,height,size=(num_of_start_goal_pairs,4))

    num_of_solutions = 0
    num_of_failed_solutions = 0
    total_time = 0.0
    failed_sol_reasons = []
    save_success_rates_to_file = False
    plot_rates = True
    
    for row in start_goals:
        start = (int(row[0]),int(row[1]))        
        goal = (int(row[2]),int(row[3]))
        # env = PointRobotWithObsAndControl(ppm_filename, obs)
        env = PointRobotWithObsAndControl(ppm_filename, obs, start, goal)
        if env.plan():
            status = str(env.setup.getLastPlannerStatus())

            if status == 'Exact solution':
                env.recordSolution(isFail=False)
                env.save(ppm_filename)
                num_of_solutions += 1
                solution = env.getSolution()
                total_time += env.getLastPathTime()
                failed_sol_reasons.append(str(env.setup.getLastPlannerStatus()))
                
            else:
                env.recordSolution(isFail=True)
                env.save(ppm_filename)
                num_of_failed_solutions += 1
                solution = env.getSolution()
                total_time += env.getLastPathTime()
                failed_sol_reasons.append(str(env.setup.getLastPlannerStatus()))
        else:
            # num_of_failed_solutions +=1
            failed_sol_reasons.append(str(env.setup.getLastPlannerStatus()))
    print(f"For configuration parameters: {Config.conf_str()}")
    print("Found {} valid solutions".format(num_of_solutions))
    print("Faild in {} solutions".format(num_of_failed_solutions))
    print(f"Total time is: {total_time} - Avrage Time is: {(total_time)/(num_of_solutions+num_of_failed_solutions)}")
    print(f"Reasons of failure: {failed_sol_reasons}")
    if save_success_rates_to_file:
        success_rates = np.load('success_rates.npy',allow_pickle=True).tolist() if os.path.isfile('success_rates.npy') else []
        time_limits = np.load('time_limits.npy',allow_pickle=True).tolist() if os.path.isfile('time_limits.npy') else []

        
        print(success_rates)
        print(time_limits)
        # success_rates.append(num_of_solutions / (num_of_solutions + num_of_failed_solutions))
        success_rates.append(num_of_solutions / (num_of_solutions + num_of_failed_solutions))
        np.save('success_rates.npy', np.array(success_rates))
        
        time_limits.append(Config.solution_time)
        # np.append(time_limits,Config.solution_time) 
        np.save('time_limits.npy', np.array(time_limits))
        print(success_rates)
        print(time_limits)
    if plot_rates:
        success_rates = np.load('success_rates.npy',allow_pickle=True).tolist() if os.path.isfile('success_rates.npy') else []
        time_limits = np.load('time_limits.npy',allow_pickle=True).tolist() if os.path.isfile('time_limits.npy') else []
        print(success_rates)
        print(time_limits)
        plt.plot(time_limits,[rate*100 for rate in success_rates])
        plt.title('Average Sucess Rates Over Time')
        plt.xlabel("Solution Time Limit [Sec]")
        plt.show()
        
    preprocess.plot()

