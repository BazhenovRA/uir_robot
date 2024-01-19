import numpy as np
from functools import reduce

np.set_printoptions(precision=4, suppress=True)


def rotate(axis, deg):
    AXIS = ('X', 'Y', 'Z')
    axis = str(axis).upper()
    if axis not in AXIS:
        print(f"{axis} is unknown axis, should be one of {AXIS}")
        return
    rot_x = axis == 'X'
    rot_y = axis == 'Y'
    rot_z = axis == 'Z'
    rot_mat = np.array([[(np.cos(deg), 1)[rot_x], (0, -np.sin(deg))[rot_z], (0, np.sin(deg))[rot_y], 0],
                        [(0, np.sin(deg))[rot_z], (np.cos(deg), 1)[rot_y], (0, -np.sin(deg))[rot_x], 0],
                        [(0, -np.sin(deg))[rot_y], (0, np.sin(deg))[rot_x], (np.cos(deg), 1)[rot_z], 0],
                        [0, 0, 0, 1]], dtype=np.float32)
    rot_mat = np.where(np.abs(rot_mat) < 1e-10, 0, rot_mat)  # get a small value when np.cos(np.pi/2)
    return rot_mat


def trans(axis, dis):
    AXIS = ('X', 'Y', 'Z')
    axis = str(axis).upper()
    if axis not in AXIS:
        print(f"{axis} is unknown axis, should be one of {AXIS}")
        return
    trans_mat = np.eye(4)
    trans_mat[AXIS.index(axis), 3] = dis

    return trans_mat

# Метод Якоби
def get_jac(joints: np.ndarray):
    delta = 0.0001
    jac = np.zeros((16, joints.shape[0]))
    for i, joint in enumerate(joints):
        joints_m = joints.copy()
        joints_p = joints.copy()
        joints_m[i] -= delta
        joints_p[i] += delta
        Tm = fk(joints_m)
        Tp = fk(joints_p)
        jac[:, i] = (Tp - Tm).flatten() / (2 * delta)

    return jac


def ik(T_tar, joints_init=np.zeros(6), tolerance=1e-7):
    itertime = 0
    step = 0.5
    joints = joints_init.copy()
    while itertime < 1000:
        T_cur = fk(joints)
        deltaT = (T_tar - T_cur).flatten()
        error = np.linalg.norm(deltaT)
        jac = get_jac(joints)
        deltaq = np.linalg.pinv(jac) @ deltaT
        joints = joints + step * deltaq
        itertime += 1
    return joints

# Высчитывание матрицы end effector
def fk(joints):
    joints = list(joints)
    thea_1 = joints[0]
    thea_2 = joints[1]
    thea_3 = joints[2]
    thea_4 = joints[3]
    thea_5 = joints[4]
    thea_6 = joints[5]



    # DH_PRAMATER: [LINK, A, D, THEA], pay attention to the unit here is M
    DH = np.array([[thea_1, 0.4, 0.18, np.pi / 2],
                   [thea_2, 0.135, 0.6, np.pi],
                   [thea_3, 0.135, 0.12, - np.pi / 2],
                   [thea_4, 0.62, 0, np.pi / 2],
                   [thea_5, 0, 0, -np.pi / 2],
                   [thea_6, 0.115, 0, 0]])
    T = [rotate('z', thea_i).dot(trans('z', d_i)).dot(trans('x', l_i)).dot(rotate('x', a_i))
         for thea_i, d_i, l_i, a_i in DH]
    T60 = reduce(np.dot, T)

    return T60

# Таблица с dh parameters для дальнейшего использования
def dh_parameters(joints):
    thea_1, thea_2, thea_3, thea_4, thea_5, thea_6 = joints
    # DH_PRAMATER: [LINK, D, A, THEA]
    DH = np.array([[thea_1, 0.4, 0.18, np.pi / 2],
                   [thea_2, 0.135, 0.6, np.pi],
                   [thea_3, 0.135, 0.12, - np.pi / 2],
                   [thea_4, 0.62, 0, np.pi / 2],
                   [thea_5, 0, 0, -np.pi / 2],
                   [thea_6, 0.115, 0, 0]])
    print(joints)
    return DH


def calculating_theta1(T60, DH):
    matrix4_6 = np.array([DH[5][1] * T60[0][2],
                          DH[5][1] * T60[1][2],
                          DH[5][1] * T60[2][2]])
    matrix0_4 = [T60[0][3] - matrix4_6[0],
                 T60[1][3] - matrix4_6[1],
                 T60[2][3] - matrix4_6[2]]

    # высчитываем первый угол
    theta_1_1 = np.degrees(np.arctan2(T60[1][3] - T60[1][2] * DH[5][1], T60[0][3] - T60[0][2] * DH[5][1]))
    theta_1_2 = np.degrees(np.arctan2(T60[1][3] - T60[1][2] * DH[5][1], T60[0][3] - T60[0][2] * DH[5][1]) - np.pi)
    print(theta_1_1, theta_1_2)

    # высчитываем третий угол
    o1o4 = np.sqrt((matrix0_4[2] - DH[0][1]) ** 2 + matrix0_4[1] ** 2 + matrix0_4[0] ** 2)
    l4 = np.sqrt(DH[1][2] ** 2 + DH[2][2] ** 2)
    theta_3_1 = np.degrees(np.arccos(-(DH[1][2] ** 2 + l4 ** 2 - np.abs(o1o4) ** 2) / 2 * DH[1][2] * l4))
    theta_3_2 = np.degrees(- np.arccos(-(DH[1][2] ** 2 + l4 ** 2 - np.abs(o1o4) ** 2) / 2 * DH[1][2] * l4))

    
    # p14_1 = np.subtract(matrix0_4, np.array([[DH[1][2] * np.cos(theta_1_1)], [DH[1][2] * np.sin(theta_1_1)], [DH[1][2] * 0]]))
    # p14_2 = np.subtract(matrix0_4, np.array([[[DH[1][2] * np.cos(theta_1_2)], [DH[1][2] * np.sin(theta_1_2)], [DH[1][2] * 0]]]))
    # theta_3_1 = 270 - np.degrees(np.arccos((DH[1][2] ** 2 + l4 ** 2 - np.sqrt(np.abs(p14_1[0][0]) ** 2 + np.abs(p14_1[0][1]) ** 2 + np.abs(p14_1[0][2]) ** 2) ** 2)
    #                                        / (2 * DH[1][2] * l4)))
    # theta_3_2 = 270 - np.degrees(np.arccos((DH[1][2] ** 2 + DH[3][1] ** 2 - np.sqrt(
    #     np.abs(p14_2[0][0]) ** 2 + np.abs(p14_2[0][1]) ** 2 + np.abs(p14_2[0][2]) ** 2) ** 2)
    #                                        / (2 * DH[1][2] * DH[3][1])))
    print(theta_3_1, theta_3_2)
    # высчитываем второй угол
    O1O4 = np.array([T60[0][3] - matrix4_6[0],
            T60[1][3] - matrix4_6[1],
            T60[2][3] - matrix4_6[2] - DH[0][1]])


    betta1 = np.degrees(np.arctan2(T60[2][3] - matrix4_6[2] - DH[0][1],
                                   (T60[0][3] - matrix4_6[0]) * np.cos(theta_1_1) + (T60[1][3] - matrix4_6[1]) * np.sin(theta_1_1)))
    gamma1 = np.degrees(np.arccos(np.dot((DH[1][2]**2 + np.dot(O1O4, O1O4) - DH[3][1] ** 2), O1O4) * ((DH[1][2] * 2) ** -1)))

    theta2 = np.degrees(np.pi/ 2) - betta1 - np.sign(theta_3_1) * gamma1
    print(*theta2)


T60 = np.array([[-0.8086, 0, 0.5883, 0.08],
              [0, 1, 0, 0.1],
              [-0.5883, 0, -0.8086, 1.2],
              [0, 0, 0, 1]])
ik(T60)
fk(ik(T60))

calculating_theta1(fk(ik(T60)), dh_parameters(ik(T60)))


