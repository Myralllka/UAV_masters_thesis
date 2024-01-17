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
            file.write(f"{x_pos:.2f},{y_pos:.2f},{z:.2f},{heading:.2f}\n")


def generate_line(p1, p2, v, output_file, heading=None):
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


if __name__ == "__main__":
    # rosservice call /uav91/control_manager/goto_trajectory_start
    # rosservice call /uav91/control_manager/start_trajectory_tracking

    output_file = "../trajectories/desired_trajectory.txt"  # Output file name
    output_file_target = "../trajectories/desired_trajectory_target.txt"

    # Example usage for a circle:
    x_center = 10.0  # x-coordinate of the circle center
    y_center = 30.0  # y-coordinate of the circle center
    z_initial = 5  # Initial z-coordinate
    radius = 10  # Radius of the circular path
    linear_velocity = 2  # Desired linear velocity
    generate_circle(x_center, y_center, z_initial, radius, linear_velocity, output_file)

    # Example usage for a line:
    # p1 = np.array([30.0, 30.0, 5.0])
    # p2 = np.array([-10.0, 30.0, 5.0])
    # v = 2
    # generate_line(p1, p2, v, output_file)
    #
    # p1 = np.array([10.0, 30.0, 3.0])
    # p2 = np.array([-20.0, 30.0, 3.0])
    # v = 0.5
    # generate_line(p1, p2, v, output_file_target)
