import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    plt.rc('lines', linewidth=2.5)
    fig, ax = plt.subplots()

    for each in [0.2, 0.4, 0.6, 0.8, 1]:
        obj_size = [each] * 500  # [m] size of the uav in a real world
        m = np.linspace(0.6, 10, 500)  # [m], distance to the object

        ideal_theta = 2 * np.arctan2(obj_size, 2 * m)
        approx_theta = obj_size / m

        approx_size = 2 * m * np.tan(approx_theta / 2)
        ideal_size = 2 * m * np.tan(ideal_theta / 2)
        diff = np.abs(approx_size - ideal_size)
        percent_diff = diff * 100 / ideal_size

        # Using set_dashes() and set_capstyle() to modify dashing of an existing line.
        line1, = ax.plot(m, percent_diff, label=f'{each}m')
    plt.xticks(np.linspace(0, 10 , 21))
    plt.xlabel("Distance to the target, [m]")
    plt.ylabel("Percentage error, [%]")
    plt.title("The error between the object size estimated with and without tangent approximation")
    ax.legend()
    plt.grid()
    plt.show()
