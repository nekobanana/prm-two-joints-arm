import numpy as np
from matplotlib import pyplot as plt
from shapely import MultiPolygon
from shapely.geometry import Polygon

from plots import init_plot, plot_polygon, plot_arm_in_config_space
from spaces_operations import check_arm_reach, cartesian_to_config_space, conf_point, plot_arm_in_workspace, \
    w_obstacle_to_c_obstacle


def main():
    x = 3
    y = 7
    l1 = 10
    l2 = 10

    ax_w, ax_c = init_plot(l1, l2)

    obstacle1_cartesian = Polygon([(2, 7), (7, 9), (10, -5), (3, -4)])
    obstacle2_cartesian = Polygon([(8, 15), (10, 15), (10, 18)])
    obstacle3_cartesian = Polygon([(13, -4), (21, -5), (17, 9), (12, 7)])
    obstacle4_cartesian = Polygon([(2, 2), (4, 2), (4, 4), (2, 4)])

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
    plot_polygon(ax_w, obstacle4_cartesian, facecolor=next(color))

    plot_arm_in_workspace(ax_w, config1.x, l1, x, y)
    plot_arm_in_workspace(ax_w, config2.x, l1, x, y)

    non_admissible_configs_shapes = []
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle1_cartesian, l1, l2))
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle2_cartesian, l1, l2))
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle3_cartesian, l1, l2))
    non_admissible_configs_shapes.append(w_obstacle_to_c_obstacle(obstacle4_cartesian, l1, l2))

    color = iter(colormap)
    for o in non_admissible_configs_shapes:
        c = next(color)
        if type(o) == MultiPolygon:
            for g in o.geoms:
                plot_polygon(ax_c, g, facecolor=c)
        else:
            plot_polygon(ax_c, o, facecolor=c)
    plot_arm_in_config_space(ax_c, [config1, config2])

    plt.show()


if __name__ == "__main__":
    main()
