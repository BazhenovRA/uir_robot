from math import acos, sin, cos, sqrt

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


def get_path_points(start_point, end_point, discretization):
    path_points = []
    x_start, y_start = start_point
    x_end, y_end = end_point

    for i in range(discretization + 1):
        x = (x_end - x_start) * i / discretization + x_start
        y = (y_end - y_start) * i / discretization + y_start
        path_points.append((x, y))

    return path_points


def get_convexity(line) -> bool:
    point_1, point_2, point_3 = line

    k = (point_3[1] - point_1[1]) / (point_3[0] - point_1[0])
    b = point_1[1] - k * point_1[0]

    if point_2[1] <= k * point_2[0] + b:
        return False
    return True


def get_angles(path_points, l_1, l_2):
    lines_1, lines_2 = [], []

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

                curve = ((0, 0), (x_1, y_1), (x_2, y_2))

                if not lines_1:
                    lines_1.append(curve)
                elif not lines_2:
                    lines_2.append(curve)
                elif get_convexity(lines_1[-1]) == get_convexity(curve):
                    lines_1.append(curve)
                elif get_convexity(lines_2[-1]) == get_convexity(curve):
                    lines_2.append(curve)
            else:
                continue

    return lines_1, lines_2


def show_plot(frames: int, ax, lines_1, lines_2):
    x_points_1 = [point[0] for point in lines_1[frames - 1]]
    y_points_1 = [point[1] for point in lines_1[frames - 1]]

    x_points_2 = [point[0] for point in lines_2[frames - 1]]
    y_points_2 = [point[1] for point in lines_2[frames - 1]]

    a, = ax.plot(x_points_1, y_points_1, color='green', marker='o', markersize=7)
    b, = ax.plot(x_points_2, y_points_2, color='red', marker='o', markersize=7)

    return [a, b]


def main():
    l_1, l_2 = map(float, input('L1 L2: ').split())
    n = 2
    points = [tuple(map(float, input(f'X{i + 1} Y{i + 1}: ').split())) for i in range(n)]
    discretization = int(input('Частота дискретизации: '))
    path_points = get_path_points(points[0], points[1], discretization)

    lines_1, lines_2 = get_angles(path_points, l_1, l_2)
    print(len(lines_1), len(lines_2))

    fig, ax = plt.subplots()
    ax.set_xlim([-1, 15])
    ax.set_ylim([-1, 15])

    animation = FuncAnimation(fig, show_plot, frames=len(lines_1),
                              fargs=(ax, lines_1, lines_2), blit=True, repeat=True, repeat_delay=2000)
    plt.show()


if __name__ == '__main__':
    main()
