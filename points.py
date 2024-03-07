import numpy as np
from shapely import Point, LineString, MultiLineString


class PrmPoint:
    def __init__(self, point: Point):
        self.point = point
        self.point_copies = [point, Point(point.x - 2 * np.pi, point.y),
                             Point(point.x + 2 * np.pi, point.y), Point(point.x, point.y - 2 * np.pi),
                             Point(point.x, point.y + 2 * np.pi), Point(point.x - 2 * np.pi, point.y - 2 * np.pi),
                             Point(point.x + 2 * np.pi, point.y + 2 * np.pi),
                             Point(point.x - 2 * np.pi, point.y + 2 * np.pi),
                             Point(point.x + 2 * np.pi, point.y - 2 * np.pi)
                             ]
        self.neighbors = []


def cut_line(line, distance, lines):
    # Cuts a line in several segments at a distance from its starting point
    if distance <= 0.0 or distance >= line.length:
        return [LineString(line)]
    coords = list(line.coords)
    for i, p in enumerate(coords):
        pd = line.project(Point(p))
        if pd == distance:
            return [
                LineString(coords[:i + 1]),
                LineString(coords[i:])
            ]
        if pd > distance:
            cp = line.interpolate(distance)
            lines.append(LineString(coords[:i] + [(cp.x, cp.y)]))
            line = LineString([(cp.x, cp.y)] + coords[i:])
            if line.length > distance:
                cut_line(line, distance, lines)
            else:
                lines.append(LineString([(cp.x, cp.y)] + coords[i:]))
            return lines


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
    reversed_list = []
    for line in reversed(lines):
        for g in reversed(list(line.geoms)):
            reversed_list.append(g.reverse())
    return MultiLineString(reversed_list)
