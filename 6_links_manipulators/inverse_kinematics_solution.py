import numpy as np
from numpy import cos, sin, arctan2, sqrt


def get_dh_params() -> dict[str, tuple]:
    our_kuka_joint_offsets = (0, 330, 1150, 115, 0, 0)  # in milimeters
    our_kuka_twist_angles = (np.pi / 2, np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, 0)  # in radians
    our_kuka_link_lengths = (645, 0, 0, 1220, 0, 240)  # in milimeters

    roboanalyzer_joint_offsets = (180, 600, 120, 0, 0, 0)  # in milimeters
    roboanalyzer_twist_angles = (np.pi / 2, np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, 0)  # in radians
    roboanalyzer_link_lengths = (400, 135, 135, 620, 0, 115)  # in milimeters

    kuka_dh_params = {'joint_offsets': our_kuka_joint_offsets,
                      'twist_angles': our_kuka_twist_angles,
                      'link_lengths': our_kuka_link_lengths}

    roboanalyzer_dh_params = {'joint_offsets': roboanalyzer_joint_offsets,
                              'twist_angles': roboanalyzer_twist_angles,
                              'link_lengths': roboanalyzer_link_lengths}

    return roboanalyzer_dh_params


def get_first_three_angles() -> list[tuple, tuple, tuple]:
    # (solution1 and solution2, solution3 and solution4, solution5 and solution6, solution7 and solution8)
    # return [(82.962, -97.038, -97.038, 82.962),
    #         (50.389, 65.658, 149.249, 139.753),
    #         (-165.649, -160.13, 2.038, 7.557)]
    return [(-117.63, 62.37, 62.37, -117.63),
            (-163.332, 175.149, 24.238, 56.722),
            (54.137, 63.286, 138.622, 147.772)]


def get_t_matrix(dh_params: dict[str, tuple], theta_1: float, theta_2: float, theta_3: float) -> list[np.ndarray]:
    matrixes = []
    angles = (theta_1, theta_2, theta_3)

    for i in range(3):
        joint_offset = dh_params['joint_offsets'][i] / 1000
        twist_angle = dh_params['twist_angles'][i]
        link_length = dh_params['link_lengths'][i] / 1000
        angle = angles[i]

        matrix_i = np.array([
            [cos(angle), -sin(angle) * cos(twist_angle), sin(angle) * sin(twist_angle), link_length * cos(angle)],
            [sin(angle), cos(angle) * cos(twist_angle), -cos(angle) * sin(twist_angle), link_length * sin(angle)],
            [0, sin(twist_angle), cos(twist_angle), joint_offset],
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
    theta_4_1 = np.degrees(arctan2(r36_matrix[1][2], r36_matrix[0][2]))
    theta_4_2 = np.degrees(arctan2(-r36_matrix[1][2], -r36_matrix[0][2]))

    theta_5_1 = np.degrees(arctan2(sqrt(r36_matrix[0][2] ** 2 + r36_matrix[1][2] ** 2), r36_matrix[2][2]))
    theta_5_2 = -theta_5_1

    theta_6_1 = np.degrees(arctan2(-r36_matrix[2][1], r36_matrix[2][0]))
    theta_6_2 = theta_6_1 + 180

    return [(theta_4_1, theta_4_2), (theta_5_1, theta_5_2), (theta_6_1, theta_6_2)]


def main():
    dh_params = get_dh_params()

    theta_1_set, theta_2_set, theta_3_set = get_first_three_angles()

    for i in range(4):  # 4 solutions as 3-DoF manipulator
        theta_1 = np.radians(theta_1_set[i])
        theta_2 = np.radians(theta_2_set[i])
        theta_3 = np.radians(theta_3_set[i])

        t01_matrix, t12_matrix, t23_matrix = get_t_matrix(dh_params, theta_1, theta_2, theta_3)

        r36_matrix = get_r36_matrix(t01_matrix, t12_matrix, t23_matrix)

        theta_4_set, theta_5_set, theta_6_set = get_last_three_angles(r36_matrix)  # 2 solutions in addition

        print(theta_4_set, theta_5_set, theta_6_set, sep='\n')
        print()


if __name__ == '__main__':
    main()
