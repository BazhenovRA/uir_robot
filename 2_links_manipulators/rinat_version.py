from math import acos, sqrt, degrees, cos, sin
from typing import Tuple, List

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


def get_angle_combinations(l_1: float, l_2: float, point: Tuple) -> List[Tuple]:
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

    return combinations


def make_paths(l_1: float, l_2: float, point: Tuple):
    line_1 = [(0, 0)]
    line_2 = [(0, 0)]

    angles = get_angle_combinations(l_1, l_2, point)

    for alpha, betta in angles:
        x_1 = l_1 * cos(alpha)
        y_1 = l_1 * sin(alpha)

        x_2 = x_1 + l_2 * cos(betta)
        y_2 = y_1 + l_2 * sin(betta)

        if abs(x_2 - point[0]) < 0.3 and abs(y_2 - point[1]) < 0.3:
            if len(line_1) == 1:
                line_1.append((x_1, y_1))
                line_1.append((x_2, y_2))
            else:
                line_2.append((x_1, y_1))
                line_2.append((x_2, y_2))

    return line_1, line_2


def get_convexity(line: List[Tuple]) -> bool:
    point_1, point_2, point_3 = line

    k = (point_3[1] - point_1[1]) / (point_3[0] - point_1[0])
    b = point_1[1] - k * point_1[0]

    if point_2[1] <= k * point_2[0] + b:
        return False
    return True


def test(frames: int, ax, discretization: int, line_1_1: List[Tuple], line_1_2: List[Tuple], line_2_1: List[Tuple],
         line_2_2: List[Tuple]):
    path_1 = []
    path_2 = []

    if get_convexity(line_1_1) == get_convexity(line_2_1):
        pair_lines = [(line_1_1, line_2_1), (line_1_2, line_2_2)]
    else:
        pair_lines = [(line_1_1, line_2_2), (line_1_2, line_2_1)]

    for pair in pair_lines:
        for i in range(3):
            point_1 = pair[0][i]
            point_2 = pair[1][i]

            delta_x = (point_2[0] - point_1[0]) / discretization
            delta_y = (point_2[1] - point_1[1]) / discretization

            if pair == pair_lines[0]:
                path_1.append((point_1[0] + delta_x * frames, point_1[1] + delta_y * frames))
            else:
                path_2.append((point_1[0] + delta_x * frames, point_1[1] + delta_y * frames))

    x_points_1 = [point[0] for point in path_1]
    y_points_1 = [point[1] for point in path_1]

    x_points_2 = [point[0] for point in path_2]
    y_points_2 = [point[1] for point in path_2]

    a, = ax.plot(x_points_1, y_points_1, color='green', marker='o', markersize=7)
    b, = ax.plot(x_points_2, y_points_2, color='red', marker='o', markersize=7)

    return [a, b]


def main():
    l_1, l_2 = map(float, input('L1 L2: ').split())
    init_point = (0, 0)
    discretization = 50
    # n = int(input('Point amount: '))
    n = 2
    points = [tuple(map(float, input(f'X{i + 1} Y{i + 1}: ').split())) for i in range(n)]

    fig, ax = plt.subplots()
    ax.set_xlim([-1, 15])
    ax.set_ylim([-1, 15])

    for i in range(len(points) - 1):
        start_point = points[i]
        end_point = points[i + 1]

        line_1_1, line_1_2 = make_paths(l_1, l_2, start_point)
        line_2_1, line_2_2 = make_paths(l_1, l_2, end_point)

        animation = FuncAnimation(fig, test, frames=discretization,
                                  fargs=(ax, discretization, line_1_1, line_1_2, line_2_1, line_2_2),
                                  interval=50, blit=True, repeat=True, repeat_delay=300)
        plt.show()


if __name__ == '__main__':
    main()
