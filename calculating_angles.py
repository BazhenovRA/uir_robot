import numpy as np


def table_dh_parameters(angles_coordinate_axis, a_measured, d_measured):
    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = angles_coordinate_axis
    a1, a2, a3, a4, a5, a6 = a_measured
    d1, d2, d3, d4, d5, d6 = d_measured
    # theta1, theta2, theta3, theta4, theta5, theta6 = angle
    # theta2 = theta2 - 90
    # theta3 = theta3 + 90
    matrix_dh = np.array([[alpha1, a1, d1],
                          [alpha2, a2, d2],
                          [alpha3, a3, d3],
                          [alpha4, a4, d4],
                          [alpha5, a5, d5],
                          [alpha6, a6, d6]])
    return matrix_dh


def angle_1(matrix_dh, target_point):
    x, y, z = target_point
    matrix_1 = np.array([0, -np.sin(matrix_dh[0][0]), np.cos(matrix_dh[0][0])])
    matrix_2 = np.array([0, -np.sin(matrix_dh[1][0]), np.cos(matrix_dh[1][0])])
    matrix_3 = np.array([0, -np.sin(matrix_dh[2][0]), np.cos(matrix_dh[2][0])])
    matrix_4 = np.array([0, -np.sin(matrix_dh[3][0]), np.cos(matrix_dh[3][0])])
    matrix_5 = np.array([0, -np.sin(matrix_dh[4][0]), np.cos(matrix_dh[4][0])])
    matrix_6 = np.array([0, -np.sin(matrix_dh[5][0]), np.cos(matrix_dh[5][0])])

    matrix_1_6 = np.array([matrix_dh[0][0], -matrix_dh[0][2] * np.sin(matrix_dh[0][0]),
                           matrix_dh[0][2] * np.cos(matrix_dh[0][0])])
    matrix_2_6 = np.array([matrix_dh[1][0], -matrix_dh[1][2] * np.sin(matrix_dh[1][0]),
                           matrix_dh[1][2] * np.cos(matrix_dh[1][0])])
    matrix_3_6 = np.array([matrix_dh[2][0], -matrix_dh[2][2] * np.sin(matrix_dh[2][0]),
                           matrix_dh[2][2] * np.cos(matrix_dh[2][0])])
    matrix_4_6 = np.array([matrix_dh[3][0], -matrix_dh[3][2] * np.sin(matrix_dh[3][0]),
                           matrix_dh[3][2] * np.cos(matrix_dh[3][0])])
    matrix_5_6 = np.array([matrix_dh[4][0], -matrix_dh[4][2] * np.sin(matrix_dh[4][0]),
                           matrix_dh[4][2] * np.cos(matrix_dh[4][0])])
    matrix_6_6 = np.array([matrix_dh[5][0], -matrix_dh[5][2] * np.sin(matrix_dh[5][0]),
                           matrix_dh[5][2] * np.cos(matrix_dh[5][0])])

    result_mul_1 = matrix_1 * matrix_2 * matrix_3 * matrix_4 * matrix_5 * matrix_6
    result_0_6 = matrix_1_6 * matrix_2_6 * matrix_3_6 * matrix_4_6 * matrix_5_6 * matrix_6_6
    res1 = matrix_dh[5][2] * result_mul_1
    total = result_0_6 - res1
    print(total, res1, result_0_6)
    # matrix_res1_1 = np.matmul(matrix_4, matrix_5, matrix_6)
    # res = np.matmul(matrix_res1_1, matrix_res1)
    # matrix_res2 = np.matmul(matrix_1_6, matrix_2_6, matrix_3_6)
    # matrix_res2_2 = np.matmul(matrix_4_6, matrix_5_6, matrix_6_6)
    # res2 = np.matmul(matrix_res2_2, matrix_res2)

    # p46 = matrix_dh[5][2] * res
    # p04 = res2 - p46
    # print(res)


def main():
    target_point = map(float, input('X Y Z: ').split())

    input_coordinate_axis = map(int, input('1 2 3 4 5 6: ').split())
    angles_coordinate_axis = []
    for elem in input_coordinate_axis:
        elem = np.degrees(np.radians(elem))
        angles_coordinate_axis.append(elem)
    a_measured = map(int, input('a1 a2 a3 a4 a5 a6: ').split())
    d_measured = map(int, input('d1 d2 d3 d4 d5 d6: ').split())
    input_angle = map(int, input('1 2 3 4 5 6: ').split())
    angle = []
    for elem in input_angle:
        elem = np.degrees(np.radians(elem))
        angle.append(elem)


angle_1(table_dh_parameters((0, -90, 0, 90, -90, -90), (0, 0, 1000, 0, 0, 0), (900, 0, 0, 300, 0, 700)),
        (1000, 500, 300))

if __name__ == '__main__':
    main()
