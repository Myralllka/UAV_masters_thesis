#!/bin/env python3
import math
import numpy as np


# import matplotlib.pyplot as plt

def generate_circle_yz(x, y, z, R, v, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the total duration of the trajectory
    circumference = 2 * math.pi * R
    total_time = circumference / v

    # Calculate the number of samples
    num_samples = int(total_time / T)

    # Calculate the center of the circle
    center_y = y
    center_z = z

    theta = 0

    # Open the output file for writing
    with open(output_file, 'w') as file:
        for i in range(num_samples):
            # Calculate the time for each sample
            # t = i * T

            # Calculate the y and z coordinates to follow a circular path
            y_pos = center_y + R * math.cos(theta)
            z_pos = center_z + R * math.sin(theta)

            # Calculate the heading angle towards the x-axis
            theta += v * T / R

            # Write the trajectory point to the file in the specified format
            file.write(f"{x:.4f},{y_pos:.4f},{z_pos:.4f},{0.0:.4f}\n")


def generate_helix_yz(x, y, z, R, v, x_length, intercircle_length, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the total duration of the trajectory
    # circumference = 2 * math.pi * R * num_subcircles
    # total_time = circumference / v

    total_time = x_length / v

    # Calculate the number of samples
    num_samples = int(total_time / T)

    # Calculate the center of the circle
    center_y = y
    center_z = z

    theta = 0

    # Open the output file for writing
    with open(output_file, 'w') as file:
        # for s in range(num_subcircles):
        for i in range(num_samples):
            # Calculate the time for each sample
            t = i * T

            # Calculate the y and z coordinates to follow a circular path
            y_pos = center_y + R * math.cos(theta)
            z_pos = center_z + R * math.sin(theta)

            x_pos = x_length * t / total_time
            # Calculate the heading angle towards the x-axis
            theta += v * T / R

            # Write the trajectory point to the file in the specified format
            file.write(f"{x_pos:.4f},{y_pos:.4f},{z_pos:.4f},{0.0:.4f}\n")


def generate_circle(x, y, z, R, v, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the total duration of the trajectory
    circumference = 2 * math.pi * R
    total_time = circumference / v

    # Calculate the number of samples
    num_samples = int(total_time / T)

    # Calculate the center of the circle
    center_x = x
    center_y = y

    theta = 0

    # Open the output file for writing
    with open(output_file, 'w') as file:
        for i in range(num_samples):
            # Calculate the time for each sample
            # t = i * T

            # Calculate the x and y coordinates to follow a circular path
            x_pos = center_x + R * math.cos(theta)
            y_pos = center_y + R * math.sin(theta)

            # Calculate the heading angle towards the center of the circle
            heading = math.atan2(center_y - y_pos, center_x - x_pos)
            theta += v * T / R

            # Write the trajectory point to the file in the specified format
            file.write(f"{x_pos:.4f},{y_pos:.4f},{z:.4f},{heading:.4f}\n")


def generate_line(p1, p2, v, output_file, heading=None):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the distance between the two points
    distance = np.linalg.norm(p2 - p1)

    # Calculate the total duration of the trajectory
    total_time = distance / v

    # Calculate the number of samples
    num_samples = int(total_time / T)
    res = []
    # Open the output file for writing
    with open(output_file, 'w') as file:
        for i in range(num_samples):
            # Calculate the time for each sample
            t = i * T

            # Interpolate between p1 and p2 based on time
            position = p1 + (p2 - p1) * (t / total_time)

            # Calculate the heading angle (pointing towards p2)
            heading = np.arctan2(p2[1] - position[1], p2[0] - position[0])

            # Write the trajectory point to the file in the specified format
            file.write(f"{position[0]:.4f},{position[1]:.4f},{position[2]:.4f},{heading:.4f}\n")
            res.append([position[0], position[1], position[2], heading])
    return res


def generate_follower(p_starting, tgt_traj, v, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2
    counter = 0
    p_eagle = p_starting
    res = []
    # Open the output file for writing
    with open(output_file, 'w') as file:
        for tgt_pos in tgt_traj:
            # Calculate the time for each sample
            t = counter * T

            velocity_vector = tgt_pos[:3] - p_eagle
            velocity = (velocity_vector / np.linalg.norm(velocity_vector)) * v
            s = velocity * T
            p_eagle_new = p_eagle + s
            # Interpolate between p1 and p2 based on time
            heading = np.arctan2(p_eagle_new[1] - p_eagle[1], p_eagle_new[0] - p_eagle[0])

            # Write the trajectory point to the file in the specified format
            file.write(f"{p_eagle_new[0]:.4f},{p_eagle_new[1]:.4f},{p_eagle_new[2]:.4f},{heading:.4f}\n")
            p_eagle = p_eagle_new
            counter += 1
    return res


def make_scenario_1():
    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"
    # Example usage for a circle:
    x_center = 0.0  # x-coordinate of the circle center
    y_center = 0.0  # y-coordinate of the circle center
    z_initial = 6  # Initial z-coordinate
    radius = 10  # Radius of the circular path
    linear_velocity = 1  # Desired linear velocity
    generate_circle(x_center, y_center, z_initial, radius, linear_velocity, output_file)
    # Example usage for a line:
    p1 = np.array([0.0, 3.0, 4])
    p2 = np.array([0.0, 3.01, 4])
    v = 0.01
    generate_line(p1, p2, v, output_file_target)


def make_scenario_2():
    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"
    # Example usage for a line:
    p1 = np.array([0.0, 0.0, 6.0])
    p2 = np.array([100.0, 0.0, 6.0])
    v = 1.5
    generate_line(p1, p2, v, output_file)

    p1 = np.array([33.0, 0.0, 4.0])
    p2 = np.array([100.0, 0.0, 4.0])
    v = 1
    generate_line(p1, p2, v, output_file_target)


def make_scenario_3():
    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"
    # Example usage for a line:
    p1 = np.array([0.0, 0.0, 4.0])
    p2 = np.array([100.0, 0.0, 4.0])
    v = 3

    l_tgt = generate_line(p1, p2, v, output_file_target)

    p_eagle_starting = np.array([-40.0, 70.0, 16.0])
    v = 4.5
    generate_follower(p_eagle_starting, l_tgt, v, output_file)


def make_scenario_4():
    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"

    x_center = 0.0  # x-coordinate of the circle center
    y_center = 0.0  # y-coordinate of the circle center
    z_initial = 10  # Initial z-coordinate
    radius = 5  # Radius of the circular path
    v = 1.5
    x_len = 100
    generate_helix_yz(x_center, y_center, z_initial, radius, v, x_len, None, output_file)

    p1 = np.array([33.0, 0.0, 12.0])
    p2 = np.array([100.0, 0.0, 12.0])
    v = 1
    generate_line(p1, p2, v, output_file_target)


def make_scenario_5():
    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"

    x_center = 0.0  # x-coordinate of the circle center
    y_center = 0.0  # y-coordinate of the circle center
    z_initial = 10  # Initial z-coordinate
    radius = 4  # Radius of the circular path
    v = 1.1
    x_len = 100
    generate_helix_yz(x_center, y_center, z_initial, radius, v, x_len, None, output_file)

    p1 = np.array([12.0, 0.0, 12.0])
    p2 = np.array([100.0, 0.0, 12.0])
    v = 1
    generate_line(p1, p2, v, output_file_target)


if __name__ == "__main__":
    # rosservice call /uav91/control_manager/goto_trajectory_start
    # rosservice call /uav91/control_manager/start_trajectory_tracking
    # rosbag record -O interceptor_faster -a -x "(.*)/image_raw/compressed(.*)" -x "(.*)/image_raw/theora(.*)"
    # rosbag record -O interceptor_faster -a -x "(.*)/image_raw/theora(.*)|(.*)/image_raw/compressed(.*)

    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"

    make_scenario_3()

    # Example usage for a circle:
    # x_center = 0.0  # x-coordinate of the circle center
    # y_center = 0.0  # y-coordinate of the circle center
    # z_initial = 4  # Initial z-coordinate
    # radius = 0  # Radius of the circular path
    # linear_velocity = 1  # Desired linear velocity
    # generate_circle(x_center, y_center, z_initial, radius, linear_velocity, output_file_target)

    # Example usage for a helix:
    # x_center = 0.0  # x-coordinate of the circle center
    # y_center = 0.0  # y-coordinate of the circle center
    # z_initial = 15  # Initial z-coordinate
    # radius = 10  # Radius of the circular path
    # v = 2
    # x_len = 200
    #
    # generate_helix_yz(x_center, y_center, z_initial, radius, v, x_len, None, output_file)

    # Example usage for a line:
    # p1 = np.array([0.0, 0.0, 6])
    # p2 = np.array([100.0, 0.0, 4.3])
    # v = 4
    # generate_line(p1, p2, v, output_file)

    # p1 = np.array([100.0, 0.0, 13.0])
    # p2 = np.array([200.0, 0.0, 13.0])
    # v = 1
    # generate_line(p1, p2, v, output_file_target)
