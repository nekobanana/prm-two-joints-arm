import numpy as np
from matplotlib import pyplot as plt


def cartesian_to_config_space(x, y, l1, l2):
    if x > 0:
        theta1, theta2 = cartesian_to_config_space_x_pos(x, y, l1, l2)
    else:
        theta1, theta2 = cartesian_to_config_space_x_pos(-x, y, l1, l2)
        theta1 = np.pi - theta1
        theta2 = np.pi - theta2
    return theta1, theta2

def cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x**2 + y**2 - l1**2 -l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan(y / x) - np.arctan((l2 * np.sin(theta2)) / (l1 + l2 * np.cos(theta2)))
    return theta1, theta2

def plot_arm(alpha, l1, l2, x, y):
    x1, y1 = l1 * np.cos(alpha), l1 * np.sin(alpha)
    plt.plot([0, x1], [0, y1], linewidth=3)
    plt.plot([x1, x], [y1, y], linewidth=3)
    ax = plt.gca()
    ax_limit = l1 + l2
    ax.axis('equal')
    ax.axhline(y=0, color='k', linewidth=1)
    ax.axvline(x=0, color='k', linewidth=1)
    ax.set(xlim=(-ax_limit, ax_limit), ylim=(-ax_limit, ax_limit))
    plt.show()

def check_arm_reach(x, y, l1, l2):
    return (x + y)**2 >= l1 and (x + y)**2 >= l2
def main():
    x = -10
    y = 5
    l1 = 10
    l2 = 10
    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    theta1, theta2 = cartesian_to_config_space(x, y, l1, l2)
    plot_arm(theta1, l1, l2, x, y)
    print(f"{theta1=}, {theta2=}")

if __name__ == "__main__":
    main()