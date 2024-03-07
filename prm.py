"""
function dijkstra(G, S)
    for each vertex V in G
        distance[V] <- infinite
        previous[V] <- NULL
        If V != S, add V to Priority Queue Q
    distance[S] <- 0

    while Q IS NOT EMPTY
        U <- Extract MIN from Q
        for each unvisited neighbour V of U
            tempDistance <- distance[U] + edge_weight(U, V)
            if tempDistance < distance[V]
                distance[V] <- tempDistance
                previous[V] <- U
    return distance[], previous[]
"""
from queue import PriorityQueue

import numpy as np
from shapely import Point, reverse

from points import PrmPoint
from spaces_operations import is_point_admissible, is_path_admissible


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
                path_admissible, trajectory = is_path_admissible(point.point, pc, obstacles)
                if path_admissible:
                    p.neighbors.append((point, trajectory))
                    point.neighbors.append((p, reverse(trajectory)))
    points.append(point)
    return point
