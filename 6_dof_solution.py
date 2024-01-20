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
    dh_table = np.array([[thea_1, 0.4, 0.18, np.pi / 2],
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
                              [-0.5883, 0, -0.8086, 1.2],
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
    print(vector_wc)

    return vector_wc


def get_first_three_angles(vector_wc, l_1, l_2, l_3) -> list[tuple, tuple, tuple, tuple]:
    """Calculate joint angles for the manipulator to reach a point."""
    x, y, z = vector_wc[0], vector_wc[1], vector_wc[2]

    theta1 = np.degrees(np.arctan2(y, x))

    z0 = 0
    x0, y0 = l_1 * np.cos(theta1), l_1 * np.sin(theta1)
    x = x - x0
    y = y - y0
    z = z - z0

    cos_theta3 = (x ** 2 + y ** 2 + z ** 2 - l_2 ** 2 - l_3 ** 2) / (2 * l_2 * l_3)
    if cos_theta3 < -1 or cos_theta3 > 1:
        return []

    theta3_1 = np.degrees(np.arccos(cos_theta3))
    theta3_2 = np.degrees(-np.arccos(cos_theta3))

    theta2_1 = np.degrees(np.arctan2(z, np.sqrt(x ** 2 + y ** 2)) - np.arctan2(l_3 * np.sin(theta3_1),
                                                                    l_2 + l_3 * np.cos(theta3_1)))
    theta2_2 = np.degrees(np.arctan2(z, np.sqrt(x ** 2 + y ** 2)) - np.arctan2(l_3 * np.sin(theta3_2),
                                                                    l_2 + l_3 * np.cos(theta3_2)))

    return [(theta1, theta2_1, theta3_1), (theta1, theta2_2, theta3_2),
            (theta1 - np.degrees(np.pi), theta2_2, theta3_2), (theta1 - np.degrees(np.pi), theta2_1, theta3_1)]


def get_r03_matrix(dh_params):
    pass


def main():
    # Input parameters
    l_1, l_2, l_3 = 400, 600, 620  # map(float, input('L1 L2: ').split())
    start_point = 7, 6, 4  # tuple(map(float, input('X1 Y1 Z1: ').split()))
    end_point = -4, 7, 3  # tuple(map(float, input('X2 Y2 Z2: ').split()))
    discretization = 200
    joints = [0, 0, 0, 0, 0, 0]
    if not can_reach_target(l_1, l_2, l_3, end_point):
        print("Введите другие параметры")
        return
    vector_wc = get_orientation_matrix_wrist_centre(get_target_matrix(), table_dh_parameters(joints))
    three_angles = get_first_three_angles(vector_wc, l_1, l_2, l_3)
    print(three_angles)
    # пока не стал цикл брать, нужно пока просто разобрраться со статичным положением
    # dh_params = get_dh_params()
    # path_points = get_path_points(start_point, end_point, discretization)

    # for point in path_points:
    #     three_angles = get_first_three_angles(point, l_1, l_2, l_3)
    #     r03_matrix = get_r03_matrix(dh_params)


if __name__ == '__main__':
    main()