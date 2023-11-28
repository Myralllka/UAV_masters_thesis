#!/bin/env python3
import math
import numpy as np


# import matplotlib.pyplot as plt


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

    # Open the output file for writing
    with open(output_file, 'w') as file:
        for i in range(num_samples):
            # Calculate the time for each sample
            t = i * T

            # Calculate the x and y coordinates to follow a circular path
            x_pos = center_x + 2 * R * math.cos(2 * math.pi * t / total_time)
            y_pos = center_y + 2 * R * math.sin(2 * math.pi * t / total_time)

            # Calculate the heading angle towards the center of the circle
            heading = math.atan2(center_y - y_pos, center_x - x_pos)

            # Write the trajectory point to the file in the specified format
            file.write(f"{x_pos:.2f},{y_pos:.2f},{z:.2f},{heading:.2f}\n")


def generate_line(p1, p2, v, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the distance between the two points
    distance = np.linalg.norm(p2 - p1)

    # Calculate the total duration of the trajectory
    total_time = distance / v

    # Calculate the number of samples
    num_samples = int(total_time / T)

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
            file.write(f"{position[0]:.2f},{position[1]:.2f},{position[2]:.2f},{heading:.2f}\n")


def generate_3d_spiral(p1, p2, r, step, v, output_file):
    # Define the sampling period T (0.2 seconds)
    T = 0.2

    # Calculate the distance between p1 and p2
    distance = np.linalg.norm(p2 - p1)

    # Calculate the total duration of the trajectory
    total_time = distance / v

    # Calculate the number of samples
    num_samples = int(total_time / T)

    # Open the output file for writing
    with open(output_file, 'w') as file:
        for i in range(num_samples):
            # Calculate the time for each sample
            t = i * T

            # Interpolate between p1 and p2 based on time
            position_on_line = p1 + (p2 - p1) * (t / total_time)

            # Calculate the spiral trajectory in 3D
            theta = 2 * math.pi * t / total_time
            x_spiral = position_on_line[0] + (r * math.cos(theta))
            y_spiral = position_on_line[1] + (r * math.sin(theta))
            z_spiral = position_on_line[2] + (t / total_time) * (p2[2] - p1[2])

            # Calculate the heading angle towards p2
            heading = np.arctan2(p2[1] - y_spiral, p2[0] - x_spiral)

            # Write the trajectory point to the file in the specified format
            file.write(f"{x_spiral:.2f},{y_spiral:.2f},{z_spiral:.2f},{heading:.2f}\n")


if __name__ == "__main__":
    # rosservice call /uav91/control_manager/goto_trajectory_start
    # rosservice call /uav91/control_manager/start_trajectory_tracking

    output_file = "../trajectories/desired_trajectory.txt"  # Output file name

    # Example usage for a circle:
    # x_center = 10.0  # x-coordinate of the circle center
    # y_center = 30.0  # y-coordinate of the circle center
    # z_initial = 3.5  # Initial z-coordinate
    # radius = 5.0  # Radius of the circular path
    # linear_velocity = 2.0  # Desired linear velocity
    # generate_circle(x_center, y_center, z_initial, radius, linear_velocity, output_file)

    # Example usage for a line:
    p1 = np.array([20.0, 30.0, 5.0])
    p2 = np.array([0.0, 30.0, 5.0])
    v = 1
    generate_line(p1, p2, v, output_file)

    # Example of usage for a spiral:
    # p1 = np.array([20.0, 30.0, 3]
    # p2 = np.array([0.0, 30.0, 3])
    # v = 1
    # r = 1
    # step=1
    # generate_3d_spiral(p1, p2, r, step, v, output_file)
