import numpy as np
from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely import Polygon, LineString


def plot_obs_in_workspace(ax, obs):
    if type(obs) is Polygon:
        ax.plot(*obs.exterior.xy)
    elif type(obs) is LineString:
        ax.plot(*obs.xy)


def plot_arm_in_config_space(ax, points):
    for p in points:
        ax.scatter(p.x, p.y, s=10)

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
    init_workspace_ax(ax1, l1, l2)
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


def init_workspace_ax(ax1, l1, l2):
    ax1_limit = l1 + l2
    ax1.set_aspect('equal', adjustable='box')
    ax1.axhline(y=0, color='k', linewidth=1)
    ax1.axvline(x=0, color='k', linewidth=1)
    ax1.set(xlim=(-ax1_limit, ax1_limit), ylim=(-ax1_limit, ax1_limit))
    ax1.set_title("Workspace")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")


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


# def plot_dijkstra_solution(ax_c, dest_config, previous):
#     p, t = previous[dest_config]
#     while p is not None:
#         for g in t.geoms:
#             ax_c.plot(*g.xy, color="red", linewidth=1)
#         p, t = previous[p]

def plot_dijkstra_solution(ax_c, solution_line):
    for g in solution_line.geoms:
        ax_c.plot(*g.xy, color="red", linewidth=1)

def plot_graph(ax_c, points):
    ax_c.scatter([p.point.x for p in points], [p.point.y for p in points], color="purple", s=5)
    for p in points:
        for n, t in p.neighbors:
            for g in t.geoms:
                ax_c.plot(*g.xy, color="black", linewidth=0.3)
