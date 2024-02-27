import numpy as np
from shapely.geometry import Point

#
#
# class WPoint:
#     def __init__(self, x, y):
#         self.point = np.array([x, y])
#
#     @property
#     def x(self):
#         return self.point[0]
#
#     @x.setter
#     def x(self, x):
#         self.point[0] = x
#
#     @property
#     def y(self):
#         return self.point[1]
#
#     @y.setter
#     def y(self, y):
#         self.point[1] = y
#
# class WObstacle:
#     def __init__(self, points: list[tuple | WPoint] | np.ndarray):
#         if type(points) is np.ndarray:
#             self.points = points
#         elif type(points) is list[WPoint]:
#             self.points = np.array(((p.x, p.y) for p in points))
#         else:
#             self.points = np.array(points)

# class CPoint(Point):
#     def __init__(self, theta1, theta2, *args, **kwargs):
#         print("aaaa")
#         super().__init__(np.mod(theta1, 2 * np.pi), np.mod(theta2, 2 * np.pi), *args, **kwargs)
#         # theta1 = np.mod(theta1, 2 * np.pi)
#         # theta2 = np.mod(theta2, 2 * np.pi)
#
#     @property
#     def theta1(self):
#         return self.x
#
#     @property
#     def theta2(self):
#         return self.y

# class CObstacle:
#     def __init__(self, points: list[tuple | CPoint] | np.ndarray):
#         if type(points) is np.ndarray:
#             self.points = points
#         elif type(points) is list:
#             self.points = np.array(((p.theta1, p.theta2) for p in points))
#         else:
#             self.points = np.array(points)
