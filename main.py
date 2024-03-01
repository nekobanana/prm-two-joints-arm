import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely import LineString, distance, geometry, normalize, union_all, MultiPolygon, box, GeometryCollection, \
    LinearRing
from shapely.geometry import Polygon, Point
from matplotlib import pyplot as plt
from shapely.ops import split

step = 0.01

def w_obstacle_to_c_obstacle(obs, l1, l2):
    non_admissible_configs_arm1 = calc_non_admissible_configs_arm1(obs, l1)
    non_admissible_configs_arm2 = calc_non_admissible_configs_arm2(obs, l1, l2, non_admissible_configs_arm1)
    na_configs_arm = []
    for theta1_interval in [*non_admissible_configs_arm1["neg"], *non_admissible_configs_arm1["pos"]]:
        if len(theta1_interval) == 0:
            continue
        shape = Polygon(((theta1_interval[0], 0), (theta1_interval[1], 0),
                         (theta1_interval[1], 2 * np.pi), (theta1_interval[0], 2 * np.pi)))
        na_configs_arm.append(shape)
    for pol in non_admissible_configs_arm2:
        shape_splitted = GeometryCollection(pol)
        try:
            shape_splitted = split(pol, box(0, 0, 2 * np.pi, 2 * np.pi).boundary)
        except ValueError:
            pass
        shape_splitted_c = []
        for g in shape_splitted.geoms:
            if g.is_empty:
                continue
            x = g.exterior.xy[0]
            y = g.exterior.xy[1]
            try:
                shape_splitted_c.append(Polygon(zip(x, y)))
            except ValueError:
                pass
        na_configs_arm.extend(shape_splitted_c)
    non_admissible_configs = union_all(na_configs_arm)
    return non_admissible_configs


def calc_non_admissible_configs_arm1(obs, l1):
    return calc_non_admissible_configs_arm(obs, 0, 0, l1, 0)


def calc_non_admissible_configs_arm(obs, center_x, center_y, l, previous_theta):
    non_admissible_configs = {"neg":[], "pos":[]}
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
        angles = []
        for point in l_intersection_coord:
            angle = np.arctan(point[1] / point[0])
            if point[0] < 0:
                angle = angle + np.pi
            angles.append(mod2pi(angle))
        angles.sort()
        if angles[0] == 0:
            angles_no_0 = [a for a in angles if a != 0]
            if 2 * np.pi - angles_no_0[-1] < angles_no_0[0]:
                angles = angles_no_0
                angles.append(2 * np.pi)
        if min(o.boundary.xy[1]) < 0:
            key = "neg"
        else:
            key = "pos"
        non_admissible_configs[key].append((angles[0], angles[-1]))
    return non_admissible_configs


def calc_non_admissible_configs_arm2(obs, l1, l2, non_admissible_configs_arm1):
    admissible_theta1 = []
    na_t1 = [*non_admissible_configs_arm1["neg"], *non_admissible_configs_arm1["pos"]]
    na_t1.sort()
    if len(non_admissible_configs_arm1["neg"]) > 0 and len(non_admissible_configs_arm1["pos"]) > 0:
        admissible_theta1 = [(na_t1[0][1], na_t1[1][0])]
    elif len(non_admissible_configs_arm1["neg"]) == 0 and len(non_admissible_configs_arm1["pos"]) == 0:
        admissible_theta1 = [(0, 2 * np.pi)]
    else:
        admissible_theta1 = [(na_t1[0][0], na_t1[0][1])]
    lines = []
    for theta1_interval in admissible_theta1:
        l = {"neg": ([], []),
             "pos": ([], [])}
        for theta1 in np.arange(theta1_interval[0], theta1_interval[1] + step, step):
            center_x = l1 * np.cos(theta1)
            center_y = l1 * np.sin(theta1)
            intervals_theta2 = calc_non_admissible_configs_arm(obs, center_x, center_y, l2, theta1)
            if len(intervals_theta2["neg"]) > 0:
                for interval in intervals_theta2["neg"]:
                    l["neg"][0].append((theta1, interval[0]))
                    l["neg"][1].append((theta1, interval[1]))
            if len(intervals_theta2["pos"]) > 0:
                for interval in intervals_theta2["pos"]:
                    l["pos"][0].append((theta1, interval[0]))
                    l["pos"][1].append((theta1, interval[1]))
            pass
        lines.append(l)
    linestrings = []
    for l in lines:
        for (k, v) in l.items():
            fig = {"l": [], "r": []}
            for idx, points in enumerate(v):
                if len(points) == 0:
                    continue
                p_it = iter(points)
                p1 = next(p_it)
                d_1 = np.inf
                i = 0
                for i, p2 in enumerate(p_it):
                    d_2 = np.linalg.norm(np.array(p2) - np.array(p1))
                    if d_2 > 5 * d_1: # numero a caso
                        break
                    d_1 = d_2
                    p1 = p2
                p_list_l = points[:i+1]
                p_list_r = points[i+1:]
                if idx == 1:
                    p_list_l = list(reversed(p_list_l))
                    p_list_r = list(reversed(p_list_r))
                fig["l"].extend(p_list_l)
                fig["r"].extend(p_list_r)
            if len(fig["l"]) > 2:
                linestrings.append(Polygon(fig["l"]))
            if len(fig["r"]) > 2:
                linestrings.append(Polygon(fig["r"]))
    return linestrings


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
    theta1_pos = np.arctan(y / x) - np.arctan(
        (l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
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

def debug_plot(l, obs):
    fig, ax = plt.subplots(1, 1)
    ax_limit = l
    ax.set_aspect('equal', adjustable='box')
    ax.axhline(y=0, color='k', linewidth=1)
    ax.axvline(x=0, color='k', linewidth=1)
    ax.set(xlim=(-ax_limit, ax_limit), ylim=(-ax_limit, ax_limit))
    ax.set_title("Transofrmed workspace")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plot_obs_in_workspace(ax, obs)
    plt.show()
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


# Plots a Polygon to pyplot `ax`
def plot_polygon(ax, poly, **kwargs):
    path = Path.make_compound_path(
        Path(np.asarray(poly.exterior.coords)[:, :2]),
        *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)

    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection
def main():
    x = 3
    y = 7
    l1 = 10
    l2 = 10

    ax_w, ax_c = init_plot(l1, l2)

    obstacle1_cartesian = Polygon([(2, 7), (7, 9), (10, -5), (3, -4)])
    obstacle2_cartesian = Polygon([(8, 15), (10, 15), (10, 18)])
    obstacle3_cartesian = Polygon([(13, -4), (21, -5), (17, 9), (12, 7)])

    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    config1 = conf_point(config1.x, config1.y)
    config2 = conf_point(config2.x, config2.y)

    colormap = plt.colormaps['Set2'].colors
    colormap = np.concatenate((np.array(colormap), 0.5 + np.zeros((len(colormap), 1))), axis=1)
    color = iter(colormap)
    plot_polygon(ax_w, obstacle1_cartesian, facecolor=next(color))
    plot_polygon(ax_w, obstacle2_cartesian, facecolor=next(color))
    plot_polygon(ax_w, obstacle3_cartesian, facecolor=next(color))

    plot_arm_in_workspace(ax_w, config1.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.x, l1, x, y)

    non_admissible_configs_shapes = []
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle1_cartesian, l1, l2))
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle2_cartesian, l1, l2))
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle3_cartesian, l1, l2))

    color = iter(colormap)
    for o in non_admissible_configs_shapes:
        c = next(color)
        for g in o.geoms:
            plot_polygon(ax_c, g, facecolor=c)
    plot_arm_in_config_space(ax_c, [config1, config2])

    plt.show()


if __name__ == "__main__":
    main()
