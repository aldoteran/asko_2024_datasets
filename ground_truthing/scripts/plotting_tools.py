import matplotlib.pyplot as plt
from matplotlib import rc
import csv

rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
rc('text', usetex=True)

def plot_trajectories():
    gps_x = []
    gps_y = []
    ins_x = []
    ins_y = []
    chopt_x = []
    chopt_y = []
    tgopt_x = []
    tgopt_y = []

    # LoLo INS.
    with open("../results/17/chaser_ins.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            ins_x.append(float(row[2]))
            ins_y.append(float(row[3]))

    # Boat gps.
    with open("../results/17/target_gps.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            gps_x.append(float(row[2]))
            gps_y.append(float(row[3]))

    # LoLo optimized and boat optimized.
    with open("../results/17/full_graph_results", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            tgopt_x.append(float(row[10]))
            tgopt_y.append(float(row[11]))
            chopt_x.append(float(row[22]))
            chopt_y.append(float(row[23]))

    fig, axs = plt.subplots()
    fig.set_figwidth(7.0)
    fig.set_figheight(7.0)

    axs.set_ylabel("northing [m]")
    axs.set_xlabel("easting [m]")
    axs.set_aspect('equal', adjustable='box')

    axs.plot(ins_x[1200:], ins_y[1200:], color='blue',
             alpha=0.6, lw=3.5, label='chaser INS')
    axs.plot(chopt_x[1200:], chopt_y[1200:], color='blue', marker='>', markevery=100,
             alpha=1.0, lw=1.0, label='chaser opt')
    axs.plot(gps_x[1200:], gps_y[1200:], color='red',
             alpha=0.6, lw=3.5, label='target GNSS')
    axs.plot(tgopt_x[1200:], tgopt_y[1200:], color='red', marker='>', markevery=100,
             alpha=1.0, lw=1.0, label='target opt')

    # axs.plot(ins_x, ins_y, color='blue', marker='s', markevery=100,
             # alpha=0.6, lw=2.0, label='chaser INS')
    # axs.plot(gps_x, gps_y, color='red', marker='o', markevery=100,
             # alpha=0.8, lw=1.0, label='target GNSS')
    # axs.plot(tgopt_x, tgopt_y, color='red', marker='^', markevery=100,
             # alpha=0.8, lw=1.0, label='target opt')
    # axs.plot(chopt_x, chopt_y, 'blue', markevery=100,
             # alpha=0.8, lw=1.0, label='chaser opt')

    # axs.plot(pf_target_y, pf_target_x, color='blue', linestyle='-.',
             # alpha=0.8, lw=1.0, label='target filtered')
    # axs.scatter(pf_target_y[-1], pf_target_x[-1], 100, edgecolor='black',
                # color='blue', marker='*', label='target filtered end')
    # axs.plot(fg_target_y, fg_target_x, color='red', linestyle='-.',
             # alpha=0.8, lw=1.0, label='target smoothed')
    # axs.scatter(fg_target_y[-1], fg_target_x[-1], 100, edgecolor='black',
                # color='red', marker='*', label='target filtered end')
    # axs.scatter(gt_target_enu['y'], gt_target_enu['x'], 100, edgecolor='black',
                # marker='*', color='green', label='target true')
    axs.grid(color='silver', linestyle=':', linewidth=0.5)

    plt.legend()
    plt.draw()

