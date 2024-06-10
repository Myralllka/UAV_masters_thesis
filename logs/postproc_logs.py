#!/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import rospy
from matplotlib.transforms import Bbox

matplotlib.use('Qt5Agg')


def set_axes_equal_3D(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def plot_3D_scene(p_eagle, p_tgt, p_est, l1, l2, l3, ax):
    # plot 3D scene

    colours = ['b', 'r', 'g']

    ax.scatter(p_eagle[0, 0], p_eagle[1, 0], p_eagle[2, 0], marker='*', c=colours[0], s=40)
    ax.scatter(p_tgt[0, 0], p_tgt[1, 0], p_tgt[2, 0], marker='*', c=colours[1], s=40)
    ax.scatter(p_est[0, 0], p_est[1, 0], p_est[2, 0], marker='*', c=colours[2], s=40)

    ax.plot(p_eagle[0, :], p_eagle[1, :], p_eagle[2, :], label=l1, c=colours[0], linewidth=1)
    ax.plot(p_tgt[0, :], p_tgt[1, :], p_tgt[2, :], label=l2, c=colours[1], linewidth=1)
    ax.plot(p_est[0, :], p_est[1, :], p_est[2, :], label=l3, c=colours[2], linewidth=1)
    ax.scatter(p_eagle[0, :], p_eagle[1, :], p_eagle[2, :], marker='.', c=colours[0], s=2)
    ax.scatter(p_tgt[0, :], p_tgt[1, :], p_tgt[2, :], marker='.', c=colours[1], s=2)
    ax.scatter(p_est[0, :], p_est[1, :], p_est[2, :], marker='.', c=colours[2], s=2)

    # Make legend, set axes limits and labels
    ax.legend(fontsize="13")
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')

    # Customize the view angle so it's easier to see that the scatter points lie
    # on the plane y=0
    ax.view_init(elev=45., azim=145, roll=0)
    return ax


def plot_nees(ax, tgt_p, est_p, est_c, l, ts):
    tmsp = [each.to_sec() for each in ts]

    dif = tgt_p - est_p
    inter = np.matmul(dif[:, np.newaxis, :], est_c)
    nees = np.matmul(inter, dif[..., np.newaxis]).reshape(-1)
    ax.plot(tmsp, nees, label=l)

    return ax


def plot_error(ax, tgt_p, est_p, l_lbl, ts):
    timestamps = [each.to_sec() for each in ts]
    values = np.linalg.norm(tgt_p - est_p, axis=0)

    # Calculate the time differences between consecutive points
    time_diffs = np.diff(timestamps)
    #
    # Identify where the difference is greater than 20 seconds
    gaps = np.where(time_diffs > 1.0)[0] + 1

    # Create a masked array
    masked_values = np.ma.masked_array(values, mask=False)
    masked_timestamps = np.ma.masked_array(timestamps, mask=False)

    # Mask the positions where there should be a gap
    masked_values.mask[gaps] = True
    masked_timestamps.mask[gaps] = True

    ax.plot(masked_timestamps, masked_values, linestyle='-', label=l_lbl)

    return ax


def plot_velocities(ax, tgt_v, est_v, l_lbl, ts):
    tmsp = [each.to_sec() for each in ts]
    err = np.linalg.norm(tgt_v - est_v, axis=0)
    # ax.plot(tmsp, err, label=l_lbl)
    ax.plot(tmsp, tgt_v, f"{l_lbl}, gt vel")
    ax.plot(tmsp, est_v, f"{l_lbl}, est vel")
    return ax


def compute_rmse(tgt_p, est_p):
    squared_diff = (est_p - tgt_p) ** 2
    mean_squared_diff = np.mean(squared_diff)
    return np.sqrt(mean_squared_diff)


def plot_error_xz(ax, tgt_p, est_p, l_lbl, ts):
    tmsp = [each.to_sec() for each in ts]
    err_yz = np.linalg.norm(tgt_p[1:3, :] - est_p[1:3, :], axis=0)

    ax.plot(tmsp, err_yz, label=l_lbl)
    return ax


def plot_dist(ax, tgt_p, est_p, eagle_p, l_lbl, ts):
    tmsp = [each.to_sec() for each in ts]
    dists = np.linalg.norm(eagle_p - tgt_p, axis=0)

    ax.plot(tmsp, dists, label=l_lbl)
    return ax


if __name__ == "__main__":
    # fname = sys.argv[1]
    # fname:
    # aproach
    # correction_threshold, m
    # detection deviation, px
    # history buf size for svd
    # real target width
    # angle variance
    # eagle position std dev, m
    # tgt init position std dev, m
    # scenario

    # each line:
    # eagle position x y z
    # eagle velocity x y z
    # target position x y z
    # target velocity x y z
    # estimated target position x y z
    # estimated target velocity x y z
    # estimated tgt size NUMBER 18
    # cov first row
    # cov second row
    # cov third row

    do_save_pdfs = True
    do_plot = True
    SCENARIOs = [1, 2, 3, 4]

    # eagle_precisions = [0.0, 0.5, 1.0, 2.0]
    # detection_precisions = [0.0, 3.0, 6.0, 9.0]
    eagle_precisions = [0.0]
    detection_precisions = [0.0]

    res = []
    for SCENARIO in SCENARIOs:
        for eagle_precision in eagle_precisions:
            for detection_precision in detection_precisions:

                fnames = [
                    # DATA
                    f"plkf_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    f"plkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    f"dkf_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    f"dkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    # f"svd-static_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    # f"svd-dynamic_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                    # CAMP
                    # 'plkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3',
                    # 'plkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3',
                    # 'dkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3',
                    # 'dkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3',
                    # 'plkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3-down',
                    # 'plkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3-down',
                    # 'dkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3-down',
                    # 'dkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-3-down',
                    # 'plkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-minidataset',
                    # 'plkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-minidataset',
                    # 'dkf_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-minidataset',
                    # 'dkft_0.0_0.0_8_0.8_0.0_0.0_0.0_real_world_corrected_angle-minidataset',
                    #     f"plkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}-additional2",
                    #     f"dkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}-additional2",
                #
                #
                ]
                # fnames = [
                #     f"plkf_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                #     f"plkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                #     f"dkf_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                #     f"dkft_0.1_{detection_precision}_8_0.4_0.0_{eagle_precision}_0.0_sc{SCENARIO}",
                # ]

                td_fig = plt.figure()
                # td_fig = plt.figure(figsize=(8, 5))
                td_fig.tight_layout()
                td_scene = td_fig.add_subplot(projection='3d')

                fig_euclidian = plt.figure(layout='constrained', figsize=(14, 9))

                subfigs = fig_euclidian.subfigures(2, 1, height_ratios=[3, 1])
                error_euclidean = subfigs[0]
                error_euclidean = error_euclidean.subplots(1, 1)
                real_distance = subfigs[1]
                real_distance = real_distance.subplots(1, 1)
                # fig, error_euclidean_only = plt.subplots(figsize=(14, 6))

                # plt.gca().set_aspect('equal')
                idx = 0
                for fname in fnames:
                    eagle_x, eagle_y, eagle_z, eagle_velx, eagle_vely, eagle_velz = [], [], [], [], [], []
                    tgt_x, tgt_y, tgt_z, tgt_velx, tgt_vely, tgt_velz = [], [], [], [], [], []
                    est_x, est_y, est_z, est_velx, est_vely, est_velz = [], [], [], [], [], []
                    est_cov = []
                    ang_est = []
                    timestamps = []
                    params = fname.split("_")
                    # fname:
                    # aproach
                    # correction_threshold, m
                    # detection deviation, px
                    # history buf size for svd
                    # real target width
                    # angle variance
                    # eagle position std dev, m
                    # tgt init position std dev, m
                    # scenario
                    method, correction_th, detection_stddev, h_bufsize, tgt_w, ang_var, eagle_stddev, tgt_init_stddev, *scenario = params

                    with open(fname, "r") as inp_f:
                        for l in inp_f.readlines():
                            l = l.split(",")
                            eagle_x.append(l[0])
                            eagle_y.append(l[1])
                            eagle_z.append(l[2])
                            eagle_velx.append([3])
                            eagle_vely.append([4])
                            eagle_velz.append([5])
                            tgt_x.append(l[6])
                            tgt_y.append(l[7])
                            tgt_z.append(l[8])
                            tgt_velx.append(l[9])
                            tgt_vely.append(l[10])
                            tgt_velz.append(l[11])
                            est_x.append(l[12])
                            est_y.append(l[13])
                            est_z.append(l[14])
                            est_velx.append(l[15])
                            est_vely.append(l[16])
                            est_velz.append(l[17])
                            ang_est.append(float(l[18]))
                            cov = np.array(list(map(float, l[19:28]))).reshape(3, 3)
                            timestamps.append(rospy.Time(int(l[28]), int(l[29])))
                            # print(f"{l[28]=}, {l[29]=}, {timestamps[-1]=}")

                            try:
                                cov_inv = np.linalg.inv(cov)
                                est_cov.append(cov_inv)
                            except Exception as e:
                                pass
                    eagle_x = np.array(list(map(float, eagle_x)))
                    eagle_y = np.array(list(map(float, eagle_y)))
                    eagle_z = np.array(list(map(float, eagle_z)))
                    eagle_velx = np.array(list(map(float, eagle_x)))
                    eagle_vely = np.array(list(map(float, eagle_y)))
                    eagle_velz = np.array(list(map(float, eagle_z)))
                    tgt_x = np.array(list(map(float, tgt_x)))
                    tgt_y = np.array(list(map(float, tgt_y)))
                    tgt_z = np.array(list(map(float, tgt_z)))
                    tgt_velx = np.array(list(map(float, tgt_x)))
                    tgt_vely = np.array(list(map(float, tgt_y)))
                    tgt_velz = np.array(list(map(float, tgt_z)))
                    est_x = np.array(list(map(float, est_x)))
                    est_y = np.array(list(map(float, est_y)))
                    est_z = np.array(list(map(float, est_z)))
                    est_velx = np.array(list(map(float, est_x)))
                    est_vely = np.array(list(map(float, est_y)))
                    est_velz = np.array(list(map(float, est_z)))
                    est_cov = np.array(est_cov)
                    eagle_pos = np.stack([eagle_x, eagle_y, eagle_z], axis=1)
                    eagle_vel = np.stack([eagle_velx, eagle_vely, eagle_velz], axis=1)
                    tgt_pos = np.stack([tgt_x, tgt_y, tgt_z], axis=1)
                    tgt_vel = np.stack([tgt_velx, tgt_vely, tgt_velz], axis=1)
                    est_pos = np.stack([est_x, est_y, est_z], axis=1)
                    est_vel = np.stack([est_velx, est_vely, est_velz], axis=1)

                    if idx == 0 and do_plot:
                        td_scene = plot_3D_scene(eagle_pos.T, tgt_pos.T, est_pos.T,
                                                 "observer, ground truth position",
                                                 "target, ground truth position",
                                                 "target, estimated position", td_scene)
                    idx += 1

                    if do_plot:
                        error_euclidean = plot_error(error_euclidean, tgt_pos.T, est_pos.T, f"{method}",
                                                     # f"eg_pres={eagle_precision},"
                                                     # f"detpres={detection_precision}",
                                                     timestamps)
                        # error_euclidean_only = plot_error(error_euclidean_only, tgt_pos.T, est_pos.T, f"{method}",
                        #                                   timestamps)
                    rmse = compute_rmse(tgt_pos.T, est_pos.T)
                    # print(f"scenario={scenario}, "
                    #       f"method={method}, "
                    #       f"eagle_pres={eagle_precision}, "
                    #       f"detection_precision={detection_precision}, "
                    #       f"rmse={rmse}")
                    print(f"{SCENARIO},"
                          f"{method},"
                          f"{eagle_precision},"
                          f"{detection_precision},"
                          f"{rmse}")

            if do_plot:
                set_axes_equal_3D(td_scene)

                #
                error_euclidean.grid(which='minor')
                error_euclidean.minorticks_on()
                error_euclidean.grid(which='major', axis='both', linestyle='-')
                error_euclidean.legend(fontsize="15", loc="upper left")
                error_euclidean.set_xlabel('time, [s]', fontsize="15")
                error_euclidean.set_ylabel('error, [m]', fontsize="15")
                #
                # fig, ax = plt.subplots()
                real_distance = plot_dist(real_distance, tgt_pos.T, est_pos.T, eagle_pos.T,
                                          "distance from the observer to the target", timestamps)
                real_distance.grid()
                real_distance.legend(fontsize="15", loc="upper right")
                real_distance.set_xlabel('time, [s]', fontsize="15")
                real_distance.set_ylabel('distance, [m]', fontsize="15")
                #
                # save pdfs
                if (do_save_pdfs):
                    fig_euclidian.savefig(f"sc_{SCENARIO}_kf_only.pdf", format="pdf", bbox_inches="tight")
                    td_fig.tight_layout()
                    td_fig.savefig(f"sc_{SCENARIO}.pdf", format="pdf",
                                   bbox_inches=Bbox([[1, 0], td_fig.get_size_inches()]))
                #
                #
                # error_euclidean_only.grid()
                # error_euclidean_only.legend()
                # error_euclidean_only.set_xlabel('time, [s]')
                # error_euclidean_only.set_ylabel('error, [m]')

                plt.show()
