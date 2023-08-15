from math import acos, asin, sin, cos, sqrt, degrees
from typing import List, Tuple

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.widgets as mwidgets


def get_path_points(start_point, end_point, discretization):
    path_points = []
    x_start, y_start = start_point
    x_end, y_end = end_point

    l, m = x_end - x_start, y_end - y_start
    for i in range(discretization + 1):
        x = l * i / discretization + x_start
        y = m * i / discretization + y_start
        path_points.append((x, y))
    return path_points


def get_angles(path_points, l_1, l_2):
    all_angles = []
    for point in path_points:

        combinations = []
        x, y = point
        sqrt_expression = sqrt(- l_1 ** 4 * l_2 ** 2 * y ** 2
                               + 2 * l_1 ** 2 * l_2 ** 4 * y ** 2
                               - l_2 ** 6 * y ** 2
                               + 2 * l_1 ** 2 * l_2 ** 2 * x ** 2 * y ** 2
                               + 2 * l_2 ** 4 * x ** 2 * y ** 2
                               - l_2 ** 2 * x ** 4 * y ** 2
                               + 2 * l_1 ** 2 * l_2 ** 2 * y ** 4
                               + 2 * l_2 ** 4 * y ** 4
                               - 2 * l_2 ** 2 * x ** 2 * y ** 4
                               - l_2 ** 2 * y ** 6
                               )

        betta_1 = acos((-l_1 ** 2 * l_2 * x + l_2 ** 3 * x + l_2 * x ** 3 + l_2 * x * y ** 2 + sqrt_expression) /
                       (2 * (l_2 ** 2 * x ** 2 + l_2 ** 2 * y ** 2)))
        betta_2 = -acos((-l_1 ** 2 * l_2 * x + l_2 ** 3 * x + l_2 * x ** 3 + l_2 * x * y ** 2 + sqrt_expression) /
                        (2 * (l_2 ** 2 * x ** 2 + l_2 ** 2 * y ** 2)))
        betta_3 = acos((-l_1 ** 2 * l_2 * x + l_2 ** 3 * x + l_2 * x ** 3 + l_2 * x * y ** 2 - sqrt_expression) /
                       (2 * (l_2 ** 2 * x ** 2 + l_2 ** 2 * y ** 2)))
        betta_4 = -acos((-l_1 ** 2 * l_2 * x + l_2 ** 3 * x + l_2 * x ** 3 + l_2 * x * y ** 2 - sqrt_expression) /
                        (2 * (l_2 ** 2 * x ** 2 + l_2 ** 2 * y ** 2)))

        for betta in {betta_1, betta_2, betta_3, betta_4}:
            combinations.append((acos((x - l_2 * cos(betta)) / l_1), betta))
            combinations.append((-acos((x - l_2 * cos(betta)) / l_1), betta))

        for alpha, betta in combinations:
            x_1 = l_1 * cos(alpha)
            y_1 = l_1 * sin(alpha)

            x_2 = x_1 + l_2 * cos(betta)
            y_2 = y_1 + l_2 * sin(betta)

            if abs(x_2 - point[0]) < 0.03 and abs(y_2 - point[1]) < 0.03:
                all_angles.append((alpha, betta))
    return all_angles


def filter_angles(all_angles):
    filtered_angles_1 = [all_angles[0]]
    filtered_angles_2 = []

    for i in range(1, len(all_angles)):
        prev_alpha, prev_betta = filtered_angles_1[-1]
        alpha, betta = all_angles[i]

        if abs(alpha - prev_alpha) < 0.0873 and abs(betta - prev_betta) < 0.0873:
            filtered_angles_1.append((alpha, betta))
        else:
            filtered_angles_2.append((alpha, betta))
    return filtered_angles_1, filtered_angles_2


def get_middle_points(path_1, path_2, l_1):
    middle_points_1 = []
    middle_points_2 = []
    for i in range(len(path_1)):
        middle_point_x_1 = l_1 * cos(path_1[i][0])
        middle_point_y_1 = l_1 * sin(path_1[i][0])
        middle_points_1.append((middle_point_x_1, middle_point_y_1))
    for i in range(len(path_2)):
        middle_point_x_2 = l_1 * cos(path_2[i][0])
        middle_point_y_2 = l_1 * sin(path_2[i][0])
        middle_points_2.append((middle_point_x_2, middle_point_y_2))
    return middle_points_1, middle_points_2


def make_lines(path_points, middle_points_1, middle_points_2):
    # TODO: (0, 0) - robot position (нужно добавить в параметры)
    line_1 = [((0, 0), middle_points_1[i], path_points[i]) for i in range(len(path_points))]
    line_2 = [((0, 0), middle_points_2[i], path_points[i]) for i in range(len(path_points))]

    return line_1, line_2


def show_plot(frames: int, ax, line_1, line_2, ):
    x_points_1, y_points_1 = zip(*line_1[frames - 1])
    x_points_2, y_points_2 = zip(*line_2[frames - 1])

    a, = ax.plot(x_points_1, y_points_1, color='green', marker='o', markersize=7)
    b, = ax.plot(x_points_2, y_points_2, color='red', marker='o', markersize=7)

    return [a, b]


def calculate_trajectory(l_1, l_2, start_point, end_point, discretization) -> None:
    path_points = get_path_points(start_point, end_point, discretization)
    all_angles = get_angles(path_points, l_1, l_2)
    path_1, path_2 = filter_angles(all_angles)

    middle_points_1, middle_points_2 = get_middle_points(path_1, path_2, l_1)

    line_1, line_2 = make_lines(path_points, middle_points_1, middle_points_2)

    fig, ax = plt.subplots()
    ax.set_xlim([-1, 15])
    ax.set_ylim([-1, 15])

    animation = FuncAnimation(fig, show_plot, frames=len(line_1), fargs=(ax, line_1, line_2),
                              interval=50, blit=True, repeat=True, repeat_delay=300)
    plt.title("Манипулятор на плоскости")
    plt.xlabel('Ось Х')
    plt.ylabel('Ось У')
    plt.legend(['line_1', ' line_2'])
    plt.grid(which='major')
    plt.show()


if __name__ == '__main__':
    # l_1, l_2 = map(float, input('L1 L2: ').split())
    # start_point = tuple(map(float, input(f'X1 Y1: ').split()))
    # end_point = tuple(map(float, input(f'X2 Y2: ').split()))
    l_1, l_2 = 10, 5
    start_point = 10.0, 10.0
    end_point = 10.0, 5.0
    discretization = 100
    calculate_trajectory(l_1, l_2, start_point, end_point, discretization)
