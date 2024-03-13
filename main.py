from random import randint

import numpy as np
from matplotlib import pyplot as plt, animation
from shapely import MultiPolygon
from shapely.geometry import Polygon

from plots import init_plot, plot_polygon, plot_arm_in_config_space, plot_dijkstra_solution, plot_graph, \
    init_workspace_ax
from points import get_solution_lines, discretize_lines
from prm import dijkstra, get_dijkstra_solution_length, add_point_to_prm, generate_prm
from spaces_operations import check_arm_reach, cartesian_to_config_space, conf_point, plot_arm_in_workspace, \
    w_obstacle_to_c_obstacle, conf_space_to_arm_position, is_point_admissible, _cartesian_to_config_space_x_pos


def main():
    """
    Works for l1 >= l2
    For l1 < l2 there is a non managed case in cartesian_to_config_space function
    (see the comment in _cartesian_to_config_space_x_pos)
    when the angle theta2 is above a certain limit.
    If theta2 remains "small" during the whole path, then it works
    (still, not encouraged)
    """
    seed = None
    # seed = 267780468
    if seed is None:
        seed = randint(0, 10**9)
    print(f"{seed=}")
    np.random.seed(seed)

    # starting point
    x, y = 7, 11
    # goal point
    x_dest, y_dest = -15, -7

    # arm lengths
    l1 = 10
    l2 = 8

    # for connecting neighbors points in PRM construction
    eps = 1
    # random points generated in PRM
    n_points = 100

    # list of Shapely polygons
    obstacles = [
        # Polygon([(2, 7), (7, 9), (10, -5), (3, -4)]),
        Polygon([(8, 10), (10, 10), (10, 13)]),
        Polygon([(10, -3), (11, -5), (10, -5)]),
        Polygon([(-10, -4), (-17, -5), (-15, 9), (-9, 7)]),
        Polygon([(2, 3), (4, 3), (4, 5), (2, 5)]),
    ]

    # not necessary, if would be caught anyway by the next try-except
    if not check_arm_reach(x, y, l1, l2):
        print("Starting point is outside arm length")
        return
    if not check_arm_reach(x_dest, y_dest, l1, l2):
        print("Goal point is outside arm length")
        return

    # get the two starting points in the configuration space corresponding to (x, y)
    try:
        config1, config2 = cartesian_to_config_space(x, y, l1, l2)
    except ValueError:
        print("Starting point cannot be reached")
        return

    # normalize the points in [0, 2pi) interval
    config1 = conf_point(config1.x, config1.y)
    config2 = conf_point(config2.x, config2.y)

    # get the two goal points in the configuration space corresponding to (x_dest, y_dest)
    try:
        dest_config1, dest_config2 = cartesian_to_config_space(x_dest, y_dest, l1, l2)
    except ValueError:
        print("Goal point cannot be reached")
        return

    # normalize the points in [0, 2pi) interval
    dest_config1 = conf_point(dest_config1.x, dest_config1.y)
    dest_config2 = conf_point(dest_config2.x, dest_config2.y)

    # colormap for pretty colors in the plots
    colormap = plt.colormaps['Set2'].colors
    colormap = np.concatenate((np.array(colormap), 0.5 + np.zeros((len(colormap), 1))), axis=1)
    color = iter(colormap)

    # initialize axes for workspace and configuration space
    ax_w, ax_c = init_plot(l1, l2)

    # plot polygons in the cartesian space
    for obstacle in obstacles:
        plot_polygon(ax_w, obstacle, facecolor=next(color))

    # populate a list with the polygons representation of the obstacles
    # in the configuration space
    non_admissible_configs_shapes = []
    for obstacle in obstacles:
        non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle, l1, l2))

    # plot the obstacles in the configuration spaces
    color = iter(colormap)
    for o in non_admissible_configs_shapes:
        c = next(color)
        if type(o) == MultiPolygon:
            for g in o.geoms:
                plot_polygon(ax_c, g, facecolor=c)
        else:
            plot_polygon(ax_c, o, facecolor=c)

    # generate the PRM points
    points = generate_prm(non_admissible_configs_shapes, eps=eps, n_points=n_points)

    # add the starting and goal configurations to th graph
    config1 = add_point_to_prm(eps, non_admissible_configs_shapes, points, config1)
    config2 = add_point_to_prm(eps, non_admissible_configs_shapes, points, config2)
    dest_config1 = add_point_to_prm(eps, non_admissible_configs_shapes, points, dest_config1)
    dest_config2 = add_point_to_prm(eps, non_admissible_configs_shapes, points, dest_config2)

    # plot the two starting and the two goal configuration of the arm
    # in the workspace (cartesian space)
    plot_arm_in_workspace(ax_w, config1.point.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.point.x, l1, x, y)
    plot_arm_in_workspace(ax_w, dest_config1.point.x, l1, x_dest, y_dest)
    plot_arm_in_workspace(ax_w, dest_config2.point.x, l1, x_dest, y_dest)
    ax_w.annotate("start", (x + 1, y - 0.5))
    ax_w.annotate("goal", (x_dest + 1, y_dest - 0.5))

    # plot the PRM in the configuration space
    plot_graph(ax_c, points)

    # plot the two starting and the two goal points in the configuration space
    plot_arm_in_config_space(ax_c, [config1.point, config2.point])
    plot_arm_in_config_space(ax_c, [dest_config1.point, dest_config2.point])

    # Check if at least one of the two starting configuration and
    # one of the two goal configurations are adissible
    if (not is_point_admissible(config1.point, non_admissible_configs_shapes)
            and not is_point_admissible(config2.point, non_admissible_configs_shapes)):
        print("Start configuration is not admissible")
        plt.show()
        return
    if (not is_point_admissible(dest_config1.point, non_admissible_configs_shapes)
            and not is_point_admissible(dest_config2.point, non_admissible_configs_shapes)):
        print("Goal configuration is not admissible")
        plt.show()
        return

    # Compute the shortest path from the two starting configurations
    # to every node in the graph with Dijkstra algorithm
    distance_1, previous_1 = dijkstra(points, config1)
    distance_2, previous_2 = dijkstra(points, config2)

    # Get the shortest solution between the four possible solutions (2 starts and 2 goals)
    shortest_path_distance, shortest_path_previous = min([
        (dest_config1, previous_1),
        (dest_config2, previous_1),
        (dest_config1, previous_2),
        (dest_config2, previous_2),
    ], key=lambda dp: get_dijkstra_solution_length(*dp))

    # get the shortest path in the form of a MultiLineString to be plotted in the
    # configuration space
    solution_line = get_solution_lines(shortest_path_distance, shortest_path_previous)

    # check if a solution exists
    if solution_line.is_empty:
        print("No solution found")
        plt.show()
        return

    # plot the solution in the graph
    plot_dijkstra_solution(ax_c, solution_line)
    plt.show()

    # Output animation path
    animation_filename = "animation.mp4"
    # The following code generates the animation (animation.mp4)
    fig = plt.figure()
    anim_ax = fig.add_subplot()
    init_workspace_ax(anim_ax, l1, l2)
    anim_ax.scatter([x, x_dest], [y, y_dest])
    anim_ax.annotate("start", (x + 1, y - 0.5))
    anim_ax.annotate("goal", (x_dest + 1, y_dest - 0.5))
    color = iter(colormap)
    for obstacle in obstacles:
        plot_polygon(anim_ax, obstacle, facecolor=next(color))

    solution_steps_discretized = discretize_lines(solution_line, 0.05)
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

    line = anim_ax.plot([], [], lw=3, color="purple")
    def animate(i):
        thisx = [0, arm1_pos_x[i], arm2_pos_x[i]]
        thisy = [0, arm1_pos_y[i], arm2_pos_y[i]]
        line[0].set_data(thisx, thisy)
        return line

    ani = animation.FuncAnimation(
        fig, animate, len(arm1_pos_x), interval=20, blit=True)
    ani.save(animation_filename, dpi=200)


if __name__ == "__main__":
    main()