import numpy as np
from matplotlib import pyplot as plt

from points import CPoint, WPoint, WObstacle


def cartesian_to_config_space(x, y, l1, l2):
    if x > 0:
        config1, config2 = _cartesian_to_config_space_x_pos(x, y, l1, l2)
    else:
        config1, config2 = _cartesian_to_config_space_x_pos(-x, y, l1, l2)
        config1.point = np.pi - config1.point
        config2.point = np.pi - config2.point
    return mod2pi(config1), mod2pi(config2)


def _cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2_pos = np.arccos(cos_theta2)
    theta2_neg = -theta2_pos
    theta1_pos = np.arctan(y / x) - np.arctan((l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
    theta1_neg = np.arctan(y / x) + np.arctan(
        (l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))  # non dovrebbe essere theta2_neg in questa formula?
    return CPoint(theta1_pos, theta2_pos), CPoint(theta1_neg, theta2_neg)


def mod2pi(c: CPoint):
    c.point = np.mod(c.point, 2 * np.pi)
    return c


def plot_arm_in_workspace(ax, alpha, l1, x, y):
    x1, y1 = l1 * np.cos(alpha), l1 * np.sin(alpha)
    ax.plot([0, x1, x], [0, y1, y], linewidth=3)


def plot_obs_in_workspace(ax, obs: WObstacle):
    points = obs.points
    points = np.concatenate((points, [points[0]]), axis=0)
    points = points.transpose()
    x = points[0]
    y = points[1]
    ax.plot(x, y)


def plot_arm_in_config_space(ax, points: list[CPoint] | CPoint):
    p = points if type(points) is list else [points]
    _plot_in_c_space(ax, p, obstacle=False)


def plot_obs_in_config_space(ax, points: list | CPoint):
    p = points if type(points) is list[CPoint] else [points]
    _plot_in_c_space(ax, p, obstacle=True)


def _plot_in_c_space(ax, points: list[CPoint], obstacle=True):
    if obstacle:
        points.append(points[0])
    theta1, theta2 = zip(*[(p.theta1, p.theta2) for p in points])
    if obstacle:
        ax.plot(theta1, theta2)
    else:
        for t1, t2 in zip(theta1, theta2):
            ax.scatter(t1, t2, marker="o")



def check_arm_reach(x, y, l1, l2):
    return (x ** 2 + y ** 2) <= (l1 + l2) ** 2


def init_plot(l1, l2):
    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1_limit = l1 + l2
    ax1.set_aspect('equal', adjustable='box')
    ax1.axhline(y=0, color='k', linewidth=1)
    ax1.axvline(x=0, color='k', linewidth=1)
    ax1.set(xlim=(-ax1_limit, ax1_limit), ylim=(-ax1_limit, ax1_limit))
    ax1.set_title("Workspace")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax2.set_title("Configuration space")
    ax2.set_xlabel("theta1")
    ax2.set_ylabel("theta2")
    ax2.set_aspect('equal', adjustable='box')
    ax2.set(xlim=(0, 2 * np.pi), ylim=(0, 2 * np.pi))
    ticks = [0, np.pi, 2 * np.pi]
    labels = [0, "pi", "2pi"]
    ax2.set_xticks(ticks, labels)
    ax2.set_yticks(ticks, labels)

    return ax1, ax2


def main():
    x = -2
    y = 5
    l1 = 10
    l2 = 10

    ax_w, ax_c = init_plot(l1, l2)

    obstacle1_cartesian = WObstacle([(3, 4), (6, 5), (5, 7)])

    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    plot_arm_in_workspace(ax_w, config1.theta1, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.theta1, l1, x, y)
    plot_obs_in_workspace(ax_w, obstacle1_cartesian)
    # cartesian_to_config_space_x_pos_sampled = []
    # obstacle1_cartesian_tmp = obstacle1_cartesian.copy()
    # obstacle1_cartesian_tmp.append(obstacle1_cartesian_tmp[0])
    # for i in range(len(obstacle1_cartesian_tmp) - 1):
    #     sampled_points = zip(np.linspace(obstacle1_cartesian_tmp[i][0], obstacle1_cartesian_tmp[i + 1][0]),
    #                       np.linspace(obstacle1_cartesian_tmp[i][1], obstacle1_cartesian_tmp[i + 1][1]))
    #     cartesian_to_config_space_x_pos_sampled += sampled_points
    # obstacle1_c_space = tuple(zip(*[cartesian_to_config_space(x_o, y_o, l1, l2)
    #                      for (x_o, y_o) in cartesian_to_config_space_x_pos_sampled]))

    plot_arm_in_config_space(ax_c, [config1, config2])
    # plot_in_c_space(ax_c, list(obstacle1_c_space[0]), obstacle=True)
    # plot_in_c_space(ax_c, list(obstacle1_c_space[1]), obstacle=True)
    plt.show()


if __name__ == "__main__":
    main()
