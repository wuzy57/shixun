# 最小冲击的生成，涉及到物理学方面

import numpy as np
import matplotlib.pyplot as plt


def minimum_jerk_trajectory(init_pos, target_pos, total_time=0.5, dt=0.01):
    xi = init_pos
    xf = target_pos
    d = total_time
    list_t = []
    list_x = []
    t = 0
    for i in range(0, int(d/dt)):
        x = xi + (xf - xi) * (10 * (t / d) ** 3 - 15 * (t / d) ** 4 + 6 * (t / d) ** 5)
        list_t.append(t)
        list_x.append(x)
        t += dt
    return np.array(list_t), np.array(list_x)


def main():
    print("Minimum Jerk Trajectory Generator")
    # This is initial position of servo motor
    init_pos = np.array([np.radians(0), np.radians(30), np.radians(20), np.radians(50)])
    # This is target position of servo motor
    target_pos = np.array([np.radians(50), np.radians(0), np.radians(50), np.radians(-20)])

    init_pos = np.array([-0.9, -0.2])
    # This is target position of servo motor
    target_pos = np.array([-0.2, -0.9])
    # We want to move all joint from initial to target position in 1 seconds
    t, x = minimum_jerk_trajectory(init_pos, target_pos, total_time=0.2)
    x = x.flatten("F")
    x_r = np.asarray([-x, -x, -x]).flatten()
    x_l = np.asarray([x, x, x]).flatten()
    pitch_trajectory = np.zeros((90, 2))
    for i in range(90):
        pitch_trajectory[i][0] = x_l[i]
        pitch_trajectory[i][1] = x_r[i+25]
    np.savetxt("./log/min_jerk_plan", pitch_trajectory, fmt='%1.4f', delimiter=' ', newline='\n')
    # Show the result
    fig, ax = plt.subplots()
    ax.plot(t, x)
    ax.set_title("Minimum Jerk Trajectory")
    ax.set_xlabel("Time")
    ax.set_ylabel("Position")
    plt.show()


if __name__ == "__main__":
    main()
