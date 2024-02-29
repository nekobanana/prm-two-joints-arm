import numpy as np
from shapely import LineString, distance, geometry, normalize, union_all, MultiPolygon, box
from shapely.geometry import Polygon, Point
from matplotlib import pyplot as plt
from shapely.ops import split


def w_obstacle_to_c_obstacle(obs, l1, l2):
    non_admissible_configs_arm1 = calc_non_admissible_configs_arm1(obs, l1)
    non_admissible_configs_arm2 = calc_non_admissible_configs_arm2(obs, l1, l2, non_admissible_configs_arm1)
    na_configs_arm = []
    for theta1_interval in non_admissible_configs_arm1:
        shape = Polygon(((theta1_interval[0], 0), (theta1_interval[1], 0),
                         (theta1_interval[1], 2 * np.pi), (theta1_interval[0], 2 * np.pi)))
        na_configs_arm.append(shape)
    for pol in non_admissible_configs_arm2:
        shape_splitted = split(pol, box(0, 0, 2 * np.pi, 2 * np.pi).boundary)
        shape_splitted_c = []
        for g in shape_splitted.geoms:
            if g.is_empty:
                continue
            # x = g.boundary.xy[0]
            # y = g.boundary.xy[1]
            x = g.xy[0]
            y = g.xy[1]
            x_neg, x_big = min(x) < 0, max(x) > 2 * np.pi
            y_neg, y_big = min(y) < 0, max(y) > 2 * np.pi
            if x_big:
                x = [mod2pi(n) for n in x if n != 2 * np.pi]
            if x_neg:
                x = [mod2pi(n) for n in x if n != 0]
            if y_big:
                y = [mod2pi(n) for n in y if n != 2 * np.pi]
            if y_neg:
                y = [mod2pi(n) for n in y if n != 0]
            # shape_splitted_c.append(Polygon(zip(x, y)))
            shape_splitted_c.append(LineString(zip(x, y)))
        na_configs_arm.extend(shape_splitted_c)
    # non_admissible_configs = MultiPolygon(na_configs_arm1)
    return na_configs_arm


def calc_non_admissible_configs_arm1(obs, l1):
    return calc_non_admissible_configs_arm(obs, 0, 0, l1)


def calc_non_admissible_configs_arm(obs, center_x, center_y, l):
    non_admissible_configs = []
    previous_theta = 0
    previous_l = 0
    if center_x != 0:
        previous_theta = np.arctan(center_y / center_x)
        # previous_l = np.sqrt(center_x ** 2 + center_y ** 2)
        transl_coords = np.array(obs.exterior.coords) - np.array([center_x, center_y])
        rot_matrix = np.array(
            [[np.cos(previous_theta), np.sin(previous_theta)],
             [-np.sin(previous_theta), np.cos(previous_theta)]])
        trasf_coords = np.matmul(rot_matrix, transl_coords.transpose()).transpose()
        obs = Polygon(trasf_coords)
    splitter = LineString(((0, 0), (max(obs.exterior.xy[0]), 0)))
    l_circle = Point(0, 0).buffer(l)
    obs_splitted = split(obs, splitter)
    for idx, o in enumerate(obs_splitted.geoms):
        intersection = l_circle.intersection(o)
        if intersection.is_empty:
            continue
        l_intersection_coord = intersection.boundary.coords
        angles = [mod2pi(np.arctan(point[1] / point[0])) for point in l_intersection_coord]
        angles.sort()
        if angles[0] == 0:
            angles_no_0 = [a for a in angles if a != 0]
            if 2 * np.pi - angles_no_0[-1] < angles_no_0[0]:
                angles = angles_no_0
                angles.append(2 * np.pi)
        # if idx == 0 and len(obs_splitted.geoms) > 0 and angles[0] == 0:
        #     angles = [a for a in angles if a != 0]
        #     angles.append(2 * np.pi)
        non_admissible_configs.append((angles[0], angles[-1]))
    return non_admissible_configs


def calc_non_admissible_configs_arm2(obs, l1, l2, non_admissible_configs_arm1):
    admissible_theta1 = []
    na_t1 = [x for xs in non_admissible_configs_arm1 for x in xs]
    na_t1.sort()
    if len(non_admissible_configs_arm1) == 2:
        admissible_theta1 = [(na_t1[1], na_t1[2])]
    elif len(non_admissible_configs_arm1) == 1:
        admissible_theta1 = [(na_t1[0], na_t1[1])]
    non_admissible_configs_arm2 = []
    polygons = []

    for theta1_interval in admissible_theta1:
        for theta1 in np.linspace(theta1_interval[0], theta1_interval[1]):
            center_x = l1 * np.cos(theta1)
            center_y = l1 * np.sin(theta1)
            intervals_theta2 = calc_non_admissible_configs_arm(obs, center_x, center_y, l2)
            polygons.extend([LineString([(theta1, inter[0]), (theta1, inter[1])]) for inter in intervals_theta2])
    # for s in non_admissible_configs_arm2:
    #     p_a = [(i[0], i[1][0]) for i in s]
    #     p_b = [(i[0], i[1][1]) for i in s]
    #     polygons.append(Polygon(p_a))
    #     polygons.append(Polygon(p_b))
    return polygons


def conf_point(theta1, theta2):
    return Point(mod2pi(theta1), mod2pi(theta2))


def mod2pi(theta):
    return np.mod(theta, 2 * np.pi)


def cartesian_to_config_space(x, y, l1, l2):
    if x > 0:
        config1, config2 = _cartesian_to_config_space_x_pos(x, y, l1, l2)
    else:
        config1, config2 = _cartesian_to_config_space_x_pos(-x, y, l1, l2)
        config1 = np.pi - np.array([config1.x, config1.y])
        config2 = np.pi - np.array([config2.x, config2.y])
    return Point(config1), Point(config2)


def _cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2_pos = np.arccos(cos_theta2)
    theta2_neg = -theta2_pos
    theta1_pos = np.arctan(y / x) - np.arctan((l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
    theta1_neg = np.arctan(y / x) + np.arctan(
        (l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))  # non dovrebbe essere theta2_neg in questa formula?
    return Point(theta1_pos, theta2_pos), Point(theta1_neg, theta2_neg)


def plot_arm_in_workspace(ax, theta1, l1, x, y):
    x1, y1 = get_arm1_position(theta1, l1)
    ax.plot([0, x1, x], [0, y1, y], linewidth=3)


def get_arm1_position(theta1, l1):
    x1, y1 = l1 * np.cos(theta1), l1 * np.sin(theta1)
    return x1, y1


def plot_obs_in_workspace(ax, obs):
    if type(obs) is Polygon:
        ax.plot(*obs.exterior.xy)
    elif type(obs) is LineString:
        ax.plot(*obs.xy)


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

    obstacle1_cartesian = Polygon([(3, -4), (11, -5), (7, 9), (2, 7)])
    # obstacle1_cartesian = Polygon([(2, 7), (7, 9), (10, 5), (3, 4)])
    # o1 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])
    # o2 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])
    # o3 = Polygon([(3, 4), (10, 5), (7, 9), (2, 7)])

    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    config1 = conf_point(config1.x, config1.y)
    config2 = conf_point(config2.x, config2.y)
    plot_arm_in_workspace(ax_w, config1.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.x, l1, x, y)
    plot_obs_in_workspace(ax_w, obstacle1_cartesian)

    plot_arm_in_config_space(ax_c, [config1, config2])
    non_admissible_configs_shapes = w_obstacle_to_c_obstacle(obstacle1_cartesian, l1, l2)
    for o in non_admissible_configs_shapes:
        plot_obs_in_workspace(ax_c, o)
    # plot_in_c_space(ax_c, list(obstacle1_c_space[0]), obstacle=True)
    # plot_in_c_space(ax_c, list(obstacle1_c_space[1]), obstacle=True)
    plt.show()


if __name__ == "__main__":
    main()
