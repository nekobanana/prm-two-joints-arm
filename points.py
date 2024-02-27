import numpy as np


class WPoint:
    def __init__(self, x, y):
        self.point = np.array([x, y])

    @property
    def x(self):
        return self.point[0]

    @x.setter
    def x(self, x):
        self.point[0] = x

    @property
    def y(self):
        return self.point[1]

    @y.setter
    def y(self, y):
        self.point[1] = y


class CPoint:
    def __init__(self, theta1, theta2):
        self.point = np.array([theta1, theta2])

    @property
    def theta1(self):
        return self.point[0]

    @theta1.setter
    def theta1(self, theta1):
        self.point[0] = theta1

    @property
    def theta2(self):
        return self.point[1]

    @theta2.setter
    def theta2(self, theta2):
        self.point[1] = theta2
