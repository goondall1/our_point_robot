height = 1024
width = 1024
num_of_start_goal_pairs = 1
radius_min = 40
radius_max = 60
distance_to_goal_tolerance = 0.5
solution_time = 1.3
propagation_step_size = 0.5
num_of_obs = 0


def conf_str():
    return f"""
    height = {height}
    width = {width}
    num_of_start_goal_pairs = {num_of_start_goal_pairs}
    radius_min = {radius_min}
    radius_max = {radius_max}
    distance_to_goal_tolerance = {distance_to_goal_tolerance}
    solution_time = {solution_time}
    propagation_step_size = {propagation_step_size}
    num_of_obs = {num_of_obs}
    """