import numpy as np
from shapely import Point, reverse, MultiLineString

from points import PrmPoint
from spaces_operations import is_point_admissible, get_path_if_admissible


def dijkstra(points: list[PrmPoint], source: PrmPoint):
    distance = {}
    previous = {}
    Q = []

    for point in points:
        distance[point] = np.inf
        previous[point] = None, None
        Q.append(point)
    distance[source] = 0
    while len(Q) > 0:
        point = min(Q, key=lambda p: distance[p])
        Q.remove(point)
        for neighbor, trajectory in point.neighbors:
            if neighbor in Q:
                tmp_d = distance[point] + trajectory.length
                if tmp_d < distance[neighbor]:
                    distance[neighbor] = tmp_d
                    previous[neighbor] = point, trajectory
    return distance, previous


def get_dijkstra_solution_length(dest_config, previous) -> float:
    l = 0
    p, t = previous[dest_config]
    while p is not None:
        l += t.length
        p, t = previous[p]
    return l if l > 0 else np.inf


def generate_prm(obstacles, eps=1.0, n_points=50):
    points = []
    for i in range(n_points):
        random_point = None
        while not is_point_admissible(random_point, obstacles):
            random_point = Point(np.random.uniform(0, 2 * np.pi, 2))
        add_point_to_prm(eps, obstacles, points, random_point)
    return points


def add_point_to_prm(eps, obstacles, points, random_point):
    point = PrmPoint(random_point)
    eps_circle = point.point.buffer(eps)
    for p in points:
        for pc in p.point_copies:
            if eps_circle.contains(pc):
                path_admissible, trajectory = get_path_if_admissible(point.point, pc, obstacles)
                if path_admissible:
                    p.neighbors.append((point, trajectory))
                    geom_reversed = list(reversed(trajectory.geoms))
                    trajectory_reversed_list = []
                    for g in geom_reversed:
                        trajectory_reversed_list.append(g.reverse())
                    point.neighbors.append((p, MultiLineString(trajectory_reversed_list)))
    points.append(point)
    return point
