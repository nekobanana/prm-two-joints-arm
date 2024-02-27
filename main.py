import numpy as np
from shapely import LineString, distance, geometry
from shapely.geometry import Polygon, Point
from matplotlib import pyplot as plt
from shapely.ops import split


def w_obstacle_to_c_obstacle(obs, l1, l2):
    non_admissible_configs = []
    l1_circle = Point(0, 0).buffer(l1)
    obs_splitted = split(obs, LineString(((0, 0), (max(obs.exterior.xy[0]), 0))))
    for idx, o in enumerate(obs_splitted.geoms):
        l1_intersection_coord = l1_circle.intersection(o).boundary.coords
        angles = [mod2pi(np.arctan(point[1] / point[0])) for point in l1_intersection_coord]
        angles.sort()
        if idx == 0 and len(obs_splitted.geoms) > 0 and angles[0] == 0:
            angles = [a for a in angles if a != 0]
            angles.append(2 * np.pi)
        non_admissible_configs.append([(angles[0], angles[-1])])
    return non_admissible_configs




def conf_point(theta1, theta2):
    return Point(mod2pi(theta1), mod2pi(theta2))


def mod2pi(theta):
    return np.mod(theta, 2 * np.pi)


def cartesian_to_config_space(x, y, l1, l2):
    if x > 0:
        config1, config2 = _cartesian_to_config_space_x_pos(x, y, l1, l2)
    else:
        config1, config2 = _cartesian_to_config_space_x_pos(-x, y, l1, l2)
        config1 = conf_point(*(np.pi - np.array([config1.x, config1.y])))
        config2 = conf_point(*(np.pi - np.array([config2.x, config2.y])))
    return config1, config2


def _cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2_pos = np.arccos(cos_theta2)
    theta2_neg = -theta2_pos
    theta1_pos = np.arctan(y / x) - np.arctan((l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
    theta1_neg = np.arctan(y / x) + np.arctan(
        (l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))  # non dovrebbe essere theta2_neg in questa formula?
    return conf_point(theta1_pos, theta2_pos), conf_point(theta1_neg, theta2_neg)

def plot_arm_in_workspace(ax, theta1, l1, x, y):
    x1, y1 = get_arm1_position(theta1, l1)
    ax.plot([0, x1, x], [0, y1, y], linewidth=3)


def get_arm1_position(theta1, l1):
    x1, y1 = l1 * np.cos(theta1), l1 * np.sin(theta1)
    return x1, y1


def plot_obs_in_workspace(ax, obs):
    ax.plot(*obs.exterior.xy)


def plot_arm_in_config_space(ax, points):
    for p in points:
        ax.scatter(p.x, p.y)


def plot_obs_in_config_space(ax, obs):
    ax.plot(*obs.exterior.xy)


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
    y = 18
    l1 = 10
    l2 = 10

    ax_w, ax_c = init_plot(l1, l2)

    obstacle1_cartesian = Polygon([(3, -4), (10, -5), (7, 9), (2, 7)])
    # obstacle1_cartesian = Polygon([(2, 7), (7, 9), (10, 5), (3, 4)])
    # o1 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])
    # o2 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])
    # o3 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])

    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    plot_arm_in_workspace(ax_w, config1.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.x, l1, x, y)
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
    w_obstacle_to_c_obstacle(obstacle1_cartesian, l1, l2)
    # plot_in_c_space(ax_c, list(obstacle1_c_space[0]), obstacle=True)
    # plot_in_c_space(ax_c, list(obstacle1_c_space[1]), obstacle=True)
    plt.show()


if __name__ == "__main__":
    main()
