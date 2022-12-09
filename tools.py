import numpy as np    


def find_average_pos(drone_list):
    average_vector = np.array([0, 0, 0])

    for drone in drone_list:
        drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
        average_vector = average_vector + drone_pos_vec
    average_vector = average_vector / len(drone_list)

    return average_vector