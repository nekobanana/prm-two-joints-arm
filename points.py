import numpy as np
from shapely import Point


class PrmPoint:
    def __init__(self, point: Point):
        self.point = point
        self.point_copies = [point, Point(point.x - 2 * np.pi, point.y),
                             Point(point.x + 2 * np.pi, point.y), Point(point.x, point.y - 2*np.pi),
                             Point(point.x, point.y + 2 * np.pi), Point(point.x - 2 * np.pi, point.y - 2*np.pi),
                             Point(point.x + 2 * np.pi, point.y + 2 * np.pi), Point(point.x - 2 * np.pi, point.y + 2*np.pi),
                             Point(point.x + 2 * np.pi, point.y - 2 * np.pi)
                            ]
        self.neighbors = []
