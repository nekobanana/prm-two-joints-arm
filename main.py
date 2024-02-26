import numpy as np
from matplotlib import pyplot as plt


def cartesian_to_config_space(x, y, l1, l2):
    if x > 0:
        (theta1_pos, theta2_pos), (theta1_neg, theta2_neg) = cartesian_to_config_space_x_pos(x, y, l1, l2)
    else:
        (theta1_pos, theta2_pos), (theta1_neg, theta2_neg) = cartesian_to_config_space_x_pos(-x, y, l1, l2)
        theta1_pos = np.pi - theta1_pos
        theta2_pos = np.pi - theta2_pos
        theta1_neg = np.pi - theta1_neg
        theta2_neg = np.pi - theta2_neg
    alpha = np.arctan(y / x)
    return (theta1_pos, theta2_pos), (theta1_neg, theta2_neg)

def cartesian_to_config_space_x_pos(x, y, l1, l2):
    cos_theta2 = (x**2 + y**2 - l1**2 -l2**2) / (2 * l1 * l2)
    theta2_pos = np.arccos(cos_theta2)
    theta2_neg = -theta2_pos
    theta1_pos = np.arctan(y / x) - np.arctan((l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
    theta1_neg = np.arctan(y / x) + np.arctan((l2 * np.sin(theta2_pos)) / (l1 + l2 * np.cos(theta2_pos)))
    return (theta1_pos, theta2_pos), (theta1_neg, theta2_neg)

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
    return (x**2 + y**2) <= (l1 + l2)**2
def main():
    x = -2
    y = 5
    l1 = 10
    l2 = 10
    if not check_arm_reach(x, y, l1, l2):
        print("Point is outside arm length")
        return
    (theta1, theta2), (theta1_2, theta2_2) = cartesian_to_config_space(x, y, l1, l2)
    plot_arm(theta1, l1, l2, x, y)
    plot_arm(theta1_2, l1, l2, x, y)
    print(f"{theta1=}, {theta2=}")

if __name__ == "__main__":
    main()