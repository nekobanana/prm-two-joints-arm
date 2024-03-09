import numpy as np
from shapely import Point, Polygon, LineString, GeometryCollection, box, MultiPolygon, MultiLineString, \
    intersection
from shapely.ops import split, unary_union

step = 0.005


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
    non_admissible_configs = unary_union(na_configs_arm)
    return non_admissible_configs.buffer(step).buffer(-step)


def calc_non_admissible_configs_arm1(obs, l1):
    return calc_non_admissible_configs_arm(obs, 0, 0, l1, 0)


def calc_non_admissible_configs_arm(obs, center_x, center_y, l, previous_theta):
    non_admissible_configs = {"neg": [], "pos": []}
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
        intersection_p = l_circle.intersection(o)
        if intersection_p.is_empty:
            continue
        l_intersection_coord = intersection_p.boundary.coords
        angles = []
        for point in l_intersection_coord:
            if point[0] != 0:
                angle = np.arctan(point[1] / point[0])
            else:
                angle = np.pi/2 if point[1] > 0 else -np.pi/2
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
    na_t1 = [*non_admissible_configs_arm1["neg"], *non_admissible_configs_arm1["pos"]]
    na_t1.sort()
    if len(non_admissible_configs_arm1["neg"]) > 0 and len(non_admissible_configs_arm1["pos"]) > 0:
        admissible_theta1 = [(na_t1[0][1], na_t1[1][0])]
    elif len(non_admissible_configs_arm1["neg"]) == 0 and len(non_admissible_configs_arm1["pos"]) == 0:
        admissible_theta1 = [(0, 2 * np.pi)]
    else:
        admissible_theta1 = [(0, na_t1[0][0]), (na_t1[0][1], 2 * np.pi)]
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
                    if d_2 > 5 * d_1:  # numero a caso
                        break
                    d_1 = d_2
                    p1 = p2
                p_list_l = points[:i + 1]
                p_list_r = points[i + 1:]
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
        config1 = np.array([np.pi, 0]) - np.array([config1.x, config1.y])
        config2 = np.array([np.pi, 0]) - np.array([config2.x, config2.y])
    return Point(config1), Point(config2)


def _cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if cos_theta2 > 1 or cos_theta2 < -1:
        raise ValueError("cos(theta2) must be between -1 and 1")
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


def check_arm_reach(x, y, l1, l2):
    return (x ** 2 + y ** 2) <= (l1 + l2) ** 2


def conf_space_to_arm_position(theta1, theta2, l1, l2):
    x1, y1 = l1 * np.cos(theta1), l1 * np.sin(theta1)
    theta_tot = mod2pi(theta2 + theta1)
    x2, y2 = l2 * np.cos(theta_tot), l2 * np.sin(theta_tot)
    x, y = x1 + x2, y1 + y2
    return (x1, y1), (x, y)


def get_path_if_admissible(start_point, end_point, obstacles):
    trajectory = LineString((start_point, end_point))
    box = MultiLineString([LineString(((-2 * np.pi, 0), (4 * np.pi, 0))),
                           LineString(((-2 * np.pi, 2 * np.pi), (4 * np.pi, 2 * np.pi))),
                           LineString(((0, -2 * np.pi), (0, 4 * np.pi))),
                           LineString(((2 * np.pi, -2 * np.pi), (2 * np.pi, 4 * np.pi)))])
    trajectory_splitted = split(trajectory, box)
    trajectory_normalized_list = []
    for t in trajectory_splitted.geoms:
        coords = t.coords
        if is_line_below_zero(t, 0):
            coords = [(2 * np.pi + x, y) for x, y in t.coords]
        elif is_line_over_2pi(t, 0):
            coords = [(x - 2 * np.pi, y) for x, y in t.coords]
        if is_line_below_zero(t, 1):
            coords = [(x, 2 * np.pi + y) for x, y in t.coords]
        elif is_line_over_2pi(t, 1):
            coords = [(x, y - 2 * np.pi) for x, y in t.coords]
        trajectory_normalized_list.append(LineString(coords))
    trajectory_normalized = MultiLineString(trajectory_normalized_list)
    for obs in obstacles:
        if not intersection(trajectory_normalized, obs).is_empty:
            return False, None
    return True, trajectory_normalized


def is_point_admissible(point, obstacles):
    if point is None:
        return False
    for obs in obstacles:
        if obs.contains(point):
            return False
    return True


def is_line_below_zero(lineString, coord_number):
    for x in lineString.xy[coord_number]:
        if x < 0:
            return True
    return False


def is_line_over_2pi(lineString, coord_number):
    for x in lineString.xy[coord_number]:
        if x > 2 * np.pi:
            return True
    return False
