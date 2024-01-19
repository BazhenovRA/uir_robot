from functools import reduce

import numpy as np
from itertools import product


def table_dh_parameters(joints):
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
    target_matrix = np.array([[-0.8086, 0, 0.5883, 0.08],
                              [0, 1, 0, 0.1],
                              [-0.5883, 0, -0.8086, 1.2],
                              [0, 0, 0, 1]])
    return target_matrix


def calculating_theta1(dh_table, target_matrix):
    x1 = target_matrix[0][3] - target_matrix[0][2] * dh_table[5][1]
    y1 = target_matrix[1][3] - target_matrix[1][2] * dh_table[5][1]

    x2 = target_matrix[0][3] - target_matrix[0][2] * dh_table[5][1]
    y2 = target_matrix[1][3] - target_matrix[1][2] * dh_table[5][1]

    theta_1_1 = np.degrees(np.arctan2(y1, x1))
    theta_1_2 = np.degrees(np.arctan2(y2, x2) - np.pi)
    print(theta_1_1, theta_1_2)
    list_theta1 = [theta_1_1, theta_1_2]
    return list_theta1


def get_matrix_vector_0_to_4(dh_table, target_matrix):
    vector0_4 = np.array([target_matrix[0][3] - dh_table[0],
                          target_matrix[1][3] - dh_table[1],
                          target_matrix[2][3] - dh_table[2]])

    return vector0_4


def get_matrix_vector_4_to_6(dh_table, target_matrix):
    vector4_6 = np.array([dh_table[5][1] * target_matrix[0][2],
                          dh_table[5][1] * target_matrix[1][2],
                          dh_table[5][1] * target_matrix[2][2]])

    return vector4_6


def calculating_theta3(dh_table, vector0_4):
    vector1_4 = np.sqrt((vector0_4[0][2] - dh_table[0][1]) ** 2 + vector0_4[0][1] ** 2 + vector0_4[0][0] ** 2)

    theta_3_1 = np.degrees(np.arccos(
        -(dh_table[1][2] ** 2 + dh_table[3][1] ** 2 - np.abs(vector1_4) ** 2) / (2 * dh_table[1][2] * dh_table[3][1])))
    theta_3_2 = np.degrees(- np.arccos(
        -(dh_table[1][2] ** 2 + dh_table[3][1] ** 2 - np.abs(vector1_4) ** 2) / (2 * dh_table[1][2] * dh_table[3][1])))

    sp_theta3 = [theta_3_1, theta_3_2]
    print(theta_3_1, theta_3_2)
    return sp_theta3


def calculating_theta2(vector0_4, dh_table, list_theta1, list_theta3):
    vector_1_4 = np.sqrt(vector0_4[0][0] ** 2 + vector0_4[0][1] ** 2 + (vector0_4[0][2] - dh_table[0][1]) ** 2)

    betta1 = np.degrees(np.arctan2(vector0_4[0][2] - dh_table[0][1],
                                   vector0_4[0][0] * np.cos(list_theta1[0] + vector0_4[0][1] * np.sin(list_theta1[0]))))
    betta2 = np.degrees(np.arctan2(vector0_4[0][2] - dh_table[0][1],
                                   vector0_4[0][0] * np.cos(list_theta1[1] + vector0_4[0][1] * np.sin(list_theta1[1]))))

    gamma = np.degrees(
        np.arccos((dh_table[1][2] ** 2 + vector_1_4 ** 2 - dh_table[3][1] ** 2) / (2 * dh_table[1][2] * vector_1_4)))

    theta_2_1 = np.degrees(np.pi / 2) - betta1 - np.sign(list_theta3[0]) * gamma
    theta_2_2 = np.degrees(np.pi / 2) - betta1 - np.sign(list_theta3[1]) * gamma
    theta_2_3 = np.degrees(np.pi / 2) - betta2 - np.sign(list_theta3[0]) * gamma
    theta_2_4 = np.degrees(np.pi / 2) - betta1 - np.sign(list_theta3[1]) * gamma

    print(theta_2_1, theta_2_2, theta_2_3, theta_2_4)

    list_theta2 = [theta_2_1, theta_2_2, theta_2_3, theta_2_4]
    return list_theta2


def get_matrix4_multiply(dh_table, list_theta1, list_theta2, list_theta3):
    theta_elementary = 0
    matrix0_1_1 = np.array([[np.cos(list_theta1[0]), -np.sin(list_theta1[0]), 0, dh_table[0][2]],
                          [np.sin(list_theta1[0]) * np.cos(dh_table[0][3]), np.cos(list_theta1[0]) * np.cos(dh_table[0][3]), -np.sin(dh_table[0][3]), -dh_table[0][1] * np.sin(dh_table[0][3])],
                          [np.sin(list_theta1[0]) * np.sin(dh_table[0][3]), np.cos(list_theta1[0]) * np.sin(dh_table[0][3]), np.cos(dh_table[0][3]), dh_table[0][1] * np.cos(dh_table[0][3])],
                          [0, 0, 0, 1]])

    matrix0_1_2 = np.array([[np.cos(list_theta1[1]), -np.sin(list_theta1[1]), 0, dh_table[0][2]],
                            [np.sin(list_theta1[1]) * np.cos(dh_table[0][3]),
                             np.cos(list_theta1[1]) * np.cos(dh_table[0][3]), -np.sin(dh_table[0][3]),
                             -dh_table[0][1] * np.sin(dh_table[0][3])],
                            [np.sin(list_theta1[1]) * np.sin(dh_table[0][3]),
                             np.cos(list_theta1[1]) * np.sin(dh_table[0][3]), np.cos(dh_table[0][3]),
                             dh_table[0][1] * np.cos(dh_table[0][3])],
                            [0, 0, 0, 1]])

    matrix1_2_1 = np.array([[np.cos(list_theta2[0]), -np.sin(list_theta2[0]), 0, dh_table[1][2]],
                          [np.sin(list_theta2[0]) * np.cos(dh_table[1][3]), np.cos(list_theta2[0]) * np.cos(dh_table[1][3]), -np.sin(dh_table[1][3]), -dh_table[1][1] * np.sin(dh_table[1][3])],
                          [np.sin(list_theta2[0]) * np.sin(dh_table[1][3]), np.cos(list_theta2[0]) * np.sin(dh_table[1][3]), np.cos(dh_table[1][3]), dh_table[1][1] * np.cos(dh_table[1][3])],
                          [0, 0, 0, 1]])

    matrix1_2_2 = np.array([[np.cos(list_theta2[1]), -np.sin(list_theta2[1]), 0, dh_table[1][2]],
                          [np.sin(list_theta2[1]) * np.cos(dh_table[1][3]),
                           np.cos(list_theta2[1]) * np.cos(dh_table[1][3]), -np.sin(dh_table[1][3]),
                           -dh_table[1][1] * np.sin(dh_table[1][3])],
                          [np.sin(list_theta2[1]) * np.sin(dh_table[1][3]),
                           np.cos(list_theta2[1]) * np.sin(dh_table[1][3]), np.cos(dh_table[1][3]),
                           dh_table[1][1] * np.cos(dh_table[1][3])],
                          [0, 0, 0, 1]])

    matrix1_2_3 = np.array([[np.cos(list_theta2[2]), -np.sin(list_theta2[2]), 0, dh_table[1][2]],
                          [np.sin(list_theta2[2]) * np.cos(dh_table[1][3]),
                           np.cos(list_theta2[2]) * np.cos(dh_table[1][3]), -np.sin(dh_table[1][3]),
                           -dh_table[1][1] * np.sin(dh_table[1][3])],
                          [np.sin(list_theta2[2]) * np.sin(dh_table[1][3]),
                           np.cos(list_theta2[2]) * np.sin(dh_table[1][3]), np.cos(dh_table[1][3]),
                           dh_table[1][1] * np.cos(dh_table[1][3])],
                          [0, 0, 0, 1]])
    matrix1_2_4 = np.array([[np.cos(list_theta2[3]), -np.sin(list_theta2[3]), 0, dh_table[1][2]],
                          [np.sin(list_theta2[3]) * np.cos(dh_table[1][3]),
                           np.cos(list_theta2[3]) * np.cos(dh_table[1][3]), -np.sin(dh_table[1][3]),
                           -dh_table[1][1] * np.sin(dh_table[1][3])],
                          [np.sin(list_theta2[3]) * np.sin(dh_table[1][3]),
                           np.cos(list_theta2[3]) * np.sin(dh_table[1][3]), np.cos(dh_table[1][3]),
                           dh_table[1][1] * np.cos(dh_table[1][3])],
                          [0, 0, 0, 1]])

    matrix2_3_1 = np.array([[np.cos(list_theta3[0]), -np.sin(list_theta3[0]), 0, dh_table[3][2]],
                          [np.sin(list_theta3[0]) * np.cos(dh_table[3][3]),
                           np.cos(list_theta3[0]) * np.cos(dh_table[3][3]), -np.sin(dh_table[3][3]),
                           -dh_table[3][1] * np.sin(dh_table[3][3])],
                          [np.sin(list_theta3[0]) * np.sin(dh_table[3][3]),
                           np.cos(list_theta3[0]) * np.sin(dh_table[3][3]), np.cos(dh_table[3][3]),
                           dh_table[3][1] * np.cos(dh_table[3][3])],
                          [0, 0, 0, 1]])

    matrix2_3_2 = np.array([[np.cos(list_theta3[1]), -np.sin(list_theta3[1]), 0, dh_table[3][2]],
                            [np.sin(list_theta3[1]) * np.cos(dh_table[3][3]),
                             np.cos(list_theta3[1]) * np.cos(dh_table[3][3]), -np.sin(dh_table[3][3]),
                             -dh_table[3][1] * np.sin(dh_table[3][3])],
                            [np.sin(list_theta3[1]) * np.sin(dh_table[3][3]),
                             np.cos(list_theta3[1]) * np.sin(dh_table[3][3]), np.cos(dh_table[3][3]),
                             dh_table[3][1] * np.cos(dh_table[3][3])],
                            [0, 0, 0, 1]])

    matrix3_4 = np.array([[np.cos(theta_elementary), -np.sin(theta_elementary), 0, dh_table[4][2]],
                            [np.sin(theta_elementary) * np.cos(dh_table[4][3]),
                             np.cos(theta_elementary) * np.cos(dh_table[4][3]), -np.sin(dh_table[4][3]),
                             -dh_table[4][1] * np.sin(dh_table[4][3])],
                            [np.sin(theta_elementary) * np.sin(dh_table[4][3]),
                             np.cos(theta_elementary) * np.sin(dh_table[4][3]), np.cos(dh_table[4][3]),
                             dh_table[4][1] * np.cos(dh_table[4][3])],
                            [0, 0, 0, 1]])










def main():
    joint = [0, 0, 0, 0, 0, 0]
    sp_theta1 = calculating_theta1(table_dh_parameters(joint), get_target_matrix())
    sp_theta3 = calculating_theta3(table_dh_parameters(joint),
                                   get_matrix_vector_0_to_4(table_dh_parameters(joint), get_target_matrix()))

    sp_theta2 = calculating_theta2(get_matrix_vector_0_to_4(table_dh_parameters(joint), get_target_matrix()), table_dh_parameters(joint),
                                   sp_theta1, sp_theta3)
    theta4 = get_matrix4_multiply(table_dh_parameters(joint), sp_theta1, sp_theta2, sp_theta3)

    # target_point = map(float, input('X Y Z: ').split())

    #
    # input_coordinate_axis = map(int, input('1 2 3 4 5 6: ').split())
    # angles_coordinate_axis = []
    # for elem in input_coordinate_axis:
    #     elem = np.degrees(np.radians(elem))
    #     angles_coordinate_axis.append(elem)
    # a_measured = map(int, input('a1 a2 a3 a4 a5 a6: ').split())
    # d_measured = map(int, input('d1 d2 d3 d4 d5 d6: ').split())
    # input_angle = map(int, input('1 2 3 4 5 6: ').split())
    # angle = []
    # for elem in input_angle:
    #     elem = np.degrees(np.radians(elem))
    #     angle.append(elem)


if __name__ == '__main__':
    main()
