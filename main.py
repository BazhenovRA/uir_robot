from math import acos, asin, sin, cos, sqrt


def get_path_points(start_point, end_point, discretization):
    path_points = []
    x_start, y_start = start_point
    x_end, y_end = end_point
    l, m = x_end - x_start, y_end - y_start
    for i in range(discretization + 1):
        x = l * i / discretization + x_start
        y = m * i / discretization + y_start
        path_points.append((x, y))
    print(path_points)
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
            else:
                continue

    print(all_angles)
    return all_angles


def get_middle_points(all_angles, l_1):
    all_middle_points = []
    for i in range(len(all_angles)):
        middle_point_x = l_1 * cos(all_angles[i][0])
        middle_point_y = l_1 * sin(all_angles[i][0])
        if (middle_point_x, middle_point_y) not in all_middle_points:
            all_middle_points.append((middle_point_x, middle_point_y))

    print(all_middle_points)
    return all_middle_points



def main():
    l1, l2 = int(input()), int(input())
    start_point = int(input()), int(input())
    end_point = int(input()), int(input())
    discretization = int(input())
    path_points = get_path_points(start_point, end_point, discretization)
    all_angles = get_angles(path_points, l1, l2)
    middle_points = get_middle_points(all_angles, l1)
main()