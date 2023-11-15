#!/bin/env python3
import math


# import matplotlib.pyplot as plt


def generate_trajectories(x, y, z, R, v, output_file):
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
        # plt.plot(center_x, center_y, ".", color="red")
        for i in range(num_samples):
            # Calculate the time for each sample
            t = i * T

            # Calculate the x and y coordinates to follow a circular path
            x_pos = center_x + 2 * R * math.cos(2 * math.pi * t / total_time)
            y_pos = center_y + 2 * R * math.sin(2 * math.pi * t / total_time)
            # plt.plot(x_pos, y_pos, ".", color='g')

            # Calculate the heading angle towards the center of the circle
            heading = math.atan2(center_y - y_pos, center_x - x_pos)

            # Write the trajectory point to the file in the specified format
            file.write(f"{x_pos:.2f},{y_pos:.2f},{z:.2f},{heading:.2f}\n")
        # plt.show()


if __name__ == "__main__":
    # rosservice call /uav91/control_manager/goto_trajectory_start
    # rosservice call /uav91/control_manager/start_trajectory_tracking
    # Example usage:
    x_center = 10.0  # x-coordinate of the circle center
    y_center = 30.0  # y-coordinate of the circle center
    z_initial = 3.5 # Initial z-coordinate
    radius = 5.0  # Radius of the circular path
    linear_velocity = 2.0  # Desired linear velocity

    output_file = "../trajectories/desired_trajectory.txt"  # Output file name

    generate_trajectories(x_center, y_center, z_initial, radius, linear_velocity, output_file)
