from matplotlib import pyplot as plt

import numpy as np


def can_reach_target(l_1, l_2, l_3, target_point):
    """Check if a three-link manipulator can reach the target point."""
    x, y, z = target_point
    l_total = l_1 + l_2 + l_3

    distance_to_target = np.linalg.norm(np.array(target_point))
    if distance_to_target > l_total:
        return False

    distance_to_base = np.linalg.norm(np.array([x, y, 0]))
    if distance_to_base > l_1 + l_2:
        return False
    if z < 0 or z > l_3:
        return False

    return True


def get_dh_params() -> dict[str, list[int]]:
    our_kuka_a_values = [0, 330, 1150, 115, 0, 0]  # in milimeters
    our_kuka_alpha_values = [np.pi / 2, np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, 0]  # in radians
    our_kuka_d_values = [645, 0, 0, 1220, 0, 240]  # in milimeters

    roboanalyzer_a_values = [180, 600, 120, 0, 0, 0]  # in milimeters
    roboanalyzer_alpha_values = [np.pi / 2, np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, 0]  # in radians
    roboanalyzer_d_values = [400, 135, 135, 620, 0, 115]  # in milimeters

    kuka_dh_params = {'joint_offsets': our_kuka_a_values,
                      'twist_angles': our_kuka_alpha_values,
                      'link_lengths': our_kuka_d_values}

    roboanalyzer_dh_params = {'joint_offsets': roboanalyzer_a_values,
                              'twist_angles': roboanalyzer_alpha_values,
                              'link_lengths': roboanalyzer_d_values}

    return roboanalyzer_dh_params


def table_dh_parameters(joints):
    """для души, мне так удобнее пока использовать"""
    thea_1, thea_2, thea_3, thea_4, thea_5, thea_6 = joints
    # DH_parameters: [LINK, D, A, THEA]
    dh_table = np.array([[thea_1, 0.645, 0.18, np.pi / 2],
                         [thea_2, 0.135, 0.6, np.pi],
                         [thea_3, 0.135, 0.12, - np.pi / 2],
                         [thea_4, 0.62, 0, np.pi / 2],
                         [thea_5, 0, 0, -np.pi / 2],
                         [thea_6, 0.115, 0, 0]])
    return dh_table


def get_target_matrix():
    """Это матрица подается на вход - начальное положение робота"""
    target_matrix = np.array([[-0.8086, 0, 0.5883, 0.08],
                              [0, 1, 0, 0.1],
                              [0.5883, 0, -0.8086, 1.2],
                              [0, 0, 0, 1]])
    return target_matrix


def get_path_points(start, end, discretization):
    """Compute linear path points between start and end."""
    x_start, y_start, z_start = start
    x_end, y_end, z_end = end
    x_values = np.linspace(x_start, x_end, discretization)
    y_values = np.linspace(y_start, y_end, discretization)
    z_values = np.linspace(z_start, z_end, discretization)
    return list(zip(x_values, y_values, z_values))


def get_orientation_matrix_wrist_centre(target_matrix, dh_table):
    """Получаем вектор для вычисления первых трех углов"""
    vector_wc = np.array([target_matrix[0][3] - dh_table[5][1] * target_matrix[0][2],
                          target_matrix[1][3] - dh_table[5][1] * target_matrix[1][2],
                          target_matrix[2][3] - dh_table[5][1] * target_matrix[2][2]])

    return vector_wc


def get_first_three_angles(vector_wc):
    """Calculate joint angles for the manipulator to reach a point."""
    x, y, z = vector_wc[0], vector_wc[1], vector_wc[2]

    theta1 = np.degrees(np.arctan2(y, x))
    theta1_1 = theta1 - 180
    a_1 = 0.18
    a_2 = 0.6
    a_3 = 0.12
    a_4 = 0.62
    d_1 = 0.4

    x0 = a_1 * np.cos(theta1 + 180)
    x0_1 = a_1 * np.cos(theta1)
    y0 = a_1 * np.sin(theta1 + 180)
    y0_1 = a_1 * np.sin(theta1)
    z0 = d_1

    l_1 = np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
    l_2 = np.sqrt(a_3 ** 2 + a_4 ** 2)

    alpha_1 = np.degrees(np.arccos((l_2 ** 2 + a_2 ** 2 - l_1 ** 2) / (2 * a_2 * l_2)))

    l_1_1 = np.sqrt((x - x0_1) ** 2 + (y - y0_1) ** 2 + (z - z0) ** 2)

    alpha_1_1 = np.degrees(np.arccos((l_2 ** 2 + a_2 ** 2 - l_1_1 ** 2) / (2 * a_2 * l_2)))

    betta_1 = np.degrees(np.arccos(a_3 / l_2))
    betta_2 = -betta_1

    theta3_1 = 180 - (alpha_1 + betta_1)
    theta3_2 = alpha_1 + betta_2 - 180
    theta3_3 = 180 - (alpha_1_1 + betta_1)
    theta3_4 = alpha_1_1 + betta_2 - 180

    angle_1_1 = np.degrees(np.arcsin((z - z0) / l_1_1))
    angle_1_2 = 180 - angle_1_1
    angle_2_1 = np.degrees(np.arccos((a_2 ** 2 + l_1_1 ** 2 - l_2 ** 2) / (2 * a_2 * l_1_1)))

    theta2_1 = angle_1_2 + angle_2_1
    theta2_2 = angle_1_2 - angle_2_1

    angle11 = np.degrees(np.arcsin((z - z0) / l_1))
    angle12 = 180 - angle11
    angle21 = np.degrees(np.arccos((a_2 ** 2 + l_1 ** 2 - l_2 ** 2) / (2 * a_2 * l_1)))

    theta2_3 = angle12 + angle21
    theta2_4 = angle12 - angle21

    sp_first_three = [(theta1, theta1_1, theta1_1, theta1), (theta2_1, theta2_3, theta2_4, theta2_2),
                      (theta3_3, theta3_1, theta3_2, theta3_4)]
    sp_first_three = [(theta1, theta1_1, theta1_1, theta1), (theta2_1, theta2_2, theta2_3, theta2_4),
                      (theta3_1, theta3_2, theta3_3, theta3_4)]

    return sp_first_three


def take_first_three_angles() -> list[tuple, tuple, tuple]:
    # (solution1 and solution2, solution3 and solution4, solution5 and solution6, solution7 and solution8)
    return [(121.463, -58.537, -58.537, 121.463),
            (175.779, 22.011, 175.779, 22.011),
            (58.33, -227.56, 58.33, -227.56)]


def get_t_matrix(dh_params: dict[str, tuple], theta_1: float, theta_2: float, theta_3: float) -> list[np.ndarray]:
    matrixes = []
    angles = (theta_1, theta_2, theta_3)

    for i in range(3):
        joint_offset = dh_params['joint_offsets'][i] / 1000
        twist_angle = dh_params['twist_angles'][i]
        link_length = dh_params['link_lengths'][i] / 1000
        angle = angles[i]

        matrix_i = np.array([
            [np.cos(angle), -np.sin(angle) * np.cos(twist_angle), np.sin(angle) * np.sin(twist_angle), link_length * np.cos(angle)],
            [np.sin(angle), np.cos(angle) * np.cos(twist_angle), -np.cos(angle) * np.sin(twist_angle), link_length * np.sin(angle)],
            [0, np.sin(twist_angle), np.cos(twist_angle), joint_offset],
            [0, 0, 0, 1]
        ])

        matrixes.append(matrix_i)

    return matrixes


def get_r36_matrix(t01_matrix: np.ndarray, t12_matrix: np.ndarray, t23_matrix: np.ndarray) -> np.ndarray:
    t03_matrix = t01_matrix.dot(t12_matrix).dot(t23_matrix)

    r03_matrix_transpose = np.array([
        [t03_matrix[0][0], t03_matrix[0][1], t03_matrix[0][2]],
        [t03_matrix[1][0], t03_matrix[1][1], t03_matrix[1][2]],
        [t03_matrix[2][0], t03_matrix[2][1], t03_matrix[2][2]]
    ]).transpose()

    orientation_matrix = np.array([
        [-0.8086, 0, 0.5883],
        [0, 1, 0],
        [-0.5883, 0, -0.8086]
    ])

    r36_matrix = r03_matrix_transpose.dot(orientation_matrix)

    return r36_matrix


def get_last_three_angles(r36_matrix: np.ndarray) -> list[tuple, tuple, tuple]:
    theta_4_1 = np.degrees(np.arctan2(r36_matrix[1][2], r36_matrix[0][2])) + 180
    theta_4_2 = np.degrees(np.arctan2(-r36_matrix[1][2], -r36_matrix[0][2])) - 180

    theta_5_1 = np.degrees(np.arctan2(np.sqrt(r36_matrix[0][2] ** 2 + r36_matrix[1][2] ** 2), r36_matrix[2][2]))
    theta_5_2 = -theta_5_1

    theta_6_1 = np.degrees(np.arctan2(-r36_matrix[2][1], r36_matrix[2][0]))
    theta_6_2 = -np.degrees(np.arctan2(-r36_matrix[2][1], -r36_matrix[2][0]))

    return [(theta_4_1, theta_4_2), (theta_5_1, theta_5_2), (theta_6_1, theta_6_2)]


def main():

    joints = [0, 0, 0, 0, 0, 0]
    dh_params = get_dh_params()
    discretization = 500
    start_point = 0.08, 0.1, 1.2
    end_point = 0.35, 0.2, 1.2

    path_points = get_path_points(start_point, end_point, discretization)
    dh_table = table_dh_parameters(joints)
    list_theta_6_1 = []
    list_theta_6_2 = []
    list_theta_6_3 = []
    list_theta_6_4 = []
    lst_path_point = [0]
    for point in path_points:
        matrix = np.array([[-0.8086, 0, 0.5883, point[0]],
                                  [0, 1, 0, point[1]],
                                  [0.5883, 0, -0.8086, point[2]],
                                  [0, 0, 0, 1]])
        vector = get_orientation_matrix_wrist_centre(matrix, dh_table)
        angles = get_first_three_angles(vector)
        print(matrix)
        print(angles)
        print('4 angles')
        for i in range(4):  # 4 solutions as 3-DoF manipulator
            theta_1 = np.radians(angles[0][i])
            theta_2 = np.radians(angles[1][i])
            theta_3 = np.radians(angles[2][i])

            t01_matrix, t12_matrix, t23_matrix = get_t_matrix(dh_params, theta_1, theta_2, theta_3)

            r36_matrix = get_r36_matrix(t01_matrix, t12_matrix, t23_matrix)

            theta_4_set, theta_5_set, theta_6_set = get_last_three_angles(r36_matrix)  # 2 solutions in addition
            if i == 0:
                list_theta_6_1.append(theta_4_set[0])
                list_theta_6_2.append(theta_4_set[1])
            # list_theta_6_3.append(theta_4_set[2])
            # list_theta_6_4.append(theta_4_set[3])
            print((theta_4_set, theta_5_set, theta_6_set))

        print('finish')
        print()

        # theta_first = angles[0][0]
        # theta_second = angles[1][1]
        # list_theta_6_1.append(theta_first)
        # list_theta_6_2.append(theta_second)
    for i in range(len(path_points) - 1):
        x_0, x_1 = path_points[0][0], path_points[i][0]
        y_0, y_1 = path_points[0][1], path_points[i][1]
        z_0, z_1 = path_points[0][2], path_points[i][2]
        d = np.sqrt((x_1 - x_0) ** 2 + (y_1 - y_0) ** 2 + (z_1 - z_0) ** 2)
        lst_path_point.append(d)
    print(lst_path_point)
    fig, ax = plt.subplots()
    ax.set_title("Зависимость углов от расстояния")
    ax.set_xlabel('Расстояние')
    ax.set_ylabel('Углы')
    ax.grid(True)
    ax.plot(lst_path_point, list_theta_6_1)
    ax.plot(lst_path_point, list_theta_6_2)
    # ax.plot(lst_path_point, list_theta_6_3)
    # ax.plot(lst_path_point, list_theta_6_4)

    plt.show()


if __name__ == '__main__':
    main()
