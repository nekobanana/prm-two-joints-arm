import dataclasses
import json
import random
from enum import Enum

import numpy as np
from matplotlib import pyplot as plt, animation
from shapely import MultiPolygon, Point, LineString, intersection, box, MultiLineString, reverse, coords
from shapely.geometry import Polygon
from shapely.ops import split

from dijkstra import dijkstra, get_dijkstra_solution_length
from plots import init_plot, plot_polygon, plot_arm_in_config_space, plot_dijkstra_solution, plot_graph, \
    init_workspace_ax
from points import PrmPoint, cut_line
from spaces_operations import check_arm_reach, cartesian_to_config_space, conf_point, plot_arm_in_workspace, \
    w_obstacle_to_c_obstacle, mod2pi


def generate_prm(obstacles, eps=1.0, n_points=50):
    points = []
    for i in range(n_points):
        random_point = None
        while not is_point_admissible(random_point, obstacles):
            random_point = Point(np.random.uniform(0, 2*np.pi, 2))
        add_point_to_prm(eps, obstacles, points, random_point)
    return points


def add_point_to_prm(eps, obstacles, points, random_point):
    point = PrmPoint(random_point)
    eps_circle = point.point.buffer(eps)
    for p in points:
        for pc in p.point_copies:
            if eps_circle.contains(pc):
                path_admissible, trajectory = is_path_admissible(point.point, pc, obstacles)
                if path_admissible:
                    p.neighbors.append((point, trajectory))
                    point.neighbors.append((p, reverse(trajectory)))
    points.append(point)
    return point


def is_line_below_zero(lineString, coord_number):
    for x in lineString.xy[coord_number]:
        if x < 0:
            return True
    return False

def is_line_over_2pi(lineString, coord_number):
    for x in lineString.xy[coord_number]:
        if x > 2*np.pi:
            return True
    return False

def is_path_admissible(start_point, end_point, obstacles):
    trajectory = LineString((start_point, end_point))
    box = MultiLineString([LineString(((-2*np.pi, 0), (4*np.pi, 0))),
                           LineString(((-2*np.pi, 2*np.pi), (4*np.pi, 2*np.pi))),
                           LineString(((0, -2*np.pi), (0, 4*np.pi))),
                           LineString(((2*np.pi, -2*np.pi), (2*np.pi, 4*np.pi)))])
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
    # trajectory_normalized = MultiLineString([LineString([(mod2pi(tx), mod2pi(ty)) for (tx, ty) in t.coords]) for t in trajectory_splitted.geoms])
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


def main():
    np.random.seed(1)

    x = 3
    y = 5
    l1 = 10
    l2 = 10

    eps = 1
    n_points = 50

    x_dest, y_dest = -8, -10

    ax_w, ax_c = init_plot(l1, l2)

    # obstacle1_cartesian = Polygon([(2, 7), (7, 9), (10, -5), (3, -4)])
    obstacle2_cartesian = Polygon([(8, 15), (10, 15), (10, 18)])
    obstacle3_cartesian = Polygon([(-10, -4), (-17, -5), (-15, 9), (-9, 7)])
    # obstacle4_cartesian = Polygon([(2, 2), (4, 2), (4, 4), (2, 4)])
    obstacles = [obstacle2_cartesian, obstacle3_cartesian]
    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    config1 = conf_point(config1.x, config1.y)
    config2 = conf_point(config2.x, config2.y)
    dest_config1, dest_config2 = cartesian_to_config_space(x_dest, y_dest, l1, l2)
    dest_config1 = conf_point(dest_config1.x, dest_config1.y)
    dest_config2 = conf_point(dest_config2.x, dest_config2.y)

    colormap = plt.colormaps['Set2'].colors
    colormap = np.concatenate((np.array(colormap), 0.5 + np.zeros((len(colormap), 1))), axis=1)
    color = iter(colormap)
    for obstacle in obstacles:
        plot_polygon(ax_w, obstacle, facecolor=next(color))

    non_admissible_configs_shapes = []
    for obstacle in obstacles:
        non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle, l1, l2))

    color = iter(colormap)
    for o in non_admissible_configs_shapes:
        c = next(color)
        if type(o) == MultiPolygon:
            for g in o.geoms:
                plot_polygon(ax_c, g, facecolor=c)
        else:
            plot_polygon(ax_c, o, facecolor=c)

    points = generate_prm(non_admissible_configs_shapes, eps=eps, n_points=n_points)
    config1 = add_point_to_prm(eps, non_admissible_configs_shapes, points, config1)
    config2 = add_point_to_prm(eps, non_admissible_configs_shapes, points, config2)
    dest_config1 = add_point_to_prm(eps, non_admissible_configs_shapes, points, dest_config1)
    dest_config2 = add_point_to_prm(eps, non_admissible_configs_shapes, points, dest_config2)

    plot_arm_in_workspace(ax_w, config1.point.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.point.x, l1, x, y)
    plot_arm_in_workspace(ax_w, dest_config1.point.x, l1, x_dest, y_dest)
    plot_arm_in_workspace(ax_w, dest_config2.point.x, l1, x_dest, y_dest)

    plot_graph(ax_c, points)

    plot_arm_in_config_space(ax_c, [config1.point, config2.point])
    plot_arm_in_config_space(ax_c, [dest_config1.point, dest_config2.point])

    distance_1, previous_1 = dijkstra(points, config1)
    distance_2, previous_2 = dijkstra(points, config2)

    shortest_path_distance, shortest_path_previous = min([
        (dest_config1, previous_1),
        (dest_config2, previous_1),
        (dest_config1, previous_2),
        (dest_config2, previous_2),
    ], key=lambda dp: get_dijkstra_solution_length(*dp))

    solution_line = get_solution_lines(shortest_path_distance, shortest_path_previous)
    plot_dijkstra_solution(ax_c, solution_line)
    plt.show()

    fig = plt.figure()
    anim_ax = fig.add_subplot()
    init_workspace_ax(anim_ax, l1, l2)
    anim_ax.scatter([x, x_dest], [y, y_dest])
    anim_ax.annotate("start", (x + 1, y - 0.5))
    anim_ax.annotate("goal", (x_dest + 1, y_dest - 0.5))
    color = iter(colormap)
    for obstacle in obstacles:
        plot_polygon(anim_ax, obstacle, facecolor=next(color))

    solution_steps_discretized = discretize_lines(solution_line, 0.1)
    arm1_pos_x = []
    arm1_pos_y = []
    arm2_pos_x = []
    arm2_pos_y = []
    for l in solution_steps_discretized:
        arm1_pos, arm2_pos = conf_space_to_arm_position(l.coords[0][0], l.coords[0][1], l1, l2)
        arm1_pos_x.append(arm1_pos[0])
        arm1_pos_y.append(arm1_pos[1])
        arm2_pos_x.append(arm2_pos[0])
        arm2_pos_y.append(arm2_pos[1])
        arm1_pos, arm2_pos = conf_space_to_arm_position(l.coords[1][0], l.coords[1][1], l1, l2)
        arm1_pos_x.append(arm1_pos[0])
        arm1_pos_y.append(arm1_pos[1])
        arm2_pos_x.append(arm2_pos[0])
        arm2_pos_y.append(arm2_pos[1])

    line = anim_ax.plot([], [], lw=2)
    def animate(i):
        thisx = [0, arm1_pos_x[i], arm2_pos_x[i]]
        thisy = [0, arm1_pos_y[i], arm2_pos_y[i]]
        line[0].set_data(thisx, thisy)
        return line

    ani = animation.FuncAnimation(
        fig, animate, len(arm1_pos_x), interval=10, blit=True)
    ani.save('animation.mp4')

def conf_space_to_arm_position(theta1, theta2, l1, l2):
    x1, y1 = l1 * np.cos(theta1), l1 * np.sin(theta1)
    # if np.pi < theta1 < 2*np.pi:
    #     x1 = -x1
    # if 1/2*np.pi < theta1 < 3/2*np.pi:
    #     y1 = -y1
    theta_tot = mod2pi(theta2 + theta1)
    x2, y2 = l2 * np.cos(theta_tot), l2 * np.sin(theta_tot)
    # if np.pi < theta_tot < 2*np.pi:
    #     x2 = -x2
    # if 1/2*np.pi < theta_tot < 3/2*np.pi:
    #     y2 = -y2
    x, y = x1 + x2, y1 + y2
    return (x1, y1), (x, y)
def discretize_lines(lines, discr_step):
    discretized = []
    for line in lines.geoms:
        discretized.extend(cut_line(line, discr_step, []))
    return discretized
def get_solution_lines(dest_config, previous):
    lines = []
    p, t = previous[dest_config]
    while p is not None:
        lines.append(t)
        p, t = previous[p]
    lines.reverse()
    reversed = []
    for line in lines:
        reversed_geoms = list(line.geoms)
        reversed_geoms.reverse()
        for g in reversed_geoms:
            reversed.append(g.reverse())
    return MultiLineString(reversed)

if __name__ == "__main__":
    main()
