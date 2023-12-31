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
    print(trans_mat)
    return trans_mat


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
        if error < tolerance:
            return joints
        jac = get_jac(joints)
        deltaq = np.linalg.pinv(jac) @ deltaT
        joints = joints + step * deltaq
        itertime += 1
    return False


def fk(joints):
    thea_1, thea_2, thea_3, thea_4, thea_5, thea_6 = joints
    # DH_PRAMATER: [LINK, A, D, THEA], pay attention to the unit here is M
    DH = np.array([[thea_1, 0.4, 0.18, 0],
                   [thea_2, 0.135, 0.6, -np.pi],
                   [thea_3, 0.135, 0.12, 0],
                   [thea_4, 0.62, 0, -np.pi / 2],
                   [thea_5, 0, 0, np.pi / 2],
                   [thea_6, 0.115, 0, -np.pi / 2]])
    T = [rotate('z', thea_i).dot(trans('z', d_i)).dot(trans('x', l_i)).dot(rotate('x', a_i))
         for thea_i, d_i, l_i, a_i in DH]
    T60 = reduce(np.dot, T)
    print(T60)
    return T60


def dh_parameters(joints):
    thea_1, thea_2, thea_3, thea_4, thea_5, thea_6 = joints
    # DH_PRAMATER: [LINK, D, A, THEA], pay attention to the unit here is M
    DH = np.array([[thea_1, 0.4, 0.18, 0],
          [thea_2, 0.135, 0.6, -np.pi],
          [thea_3, 0.135, 0.12, 0],
          [thea_4, 0.62, 0, -np.pi / 2],
          [thea_5, 0, 0, np.pi / 2],
          [thea_6, 0.115, 0, -np.pi/2]])
    return DH


def calculating_theta1(T60, DH):
    matrix4_6 = np.array([DH[5][1] * T60[0][2],
                 DH[5][1] * T60[1][2],
                 DH[5][1] * T60[2][2]])
    matrix0_4 = [T60[0][3] - matrix4_6[0],
                 T60[1][3] - matrix4_6[1],
                 T60[2][3] - matrix4_6[2]]

    theta_1_1 = np.degrees(np.arctan2(DH[1][3] - DH[1][2] * DH[5][1], DH[0][3] - DH[0][2] * DH[5][1]))
    theta_1_2 = np.degrees(np.arctan2(DH[1][3] - DH[1][2] * DH[5][1], DH[0][3] - DH[0][2] * DH[5][1]) - np.pi)
    print(theta_1_1, theta_1_2)
    sp = [theta_1_2, theta_1_1]
    return sp


def calculating_theta2(T60, DH, sp):
    matrix4_6 = [DH[5][1] * T60[0][2],
                 DH[5][1] * T60[1][2],
                 DH[5][1] * T60[2][2]]
    matrix0_4 = [T60[0][3] - matrix4_6[0],
                 T60[1][3] - matrix4_6[1],
                 T60[2][3] - matrix4_6[2]]
    o1o4 = np.sqrt((matrix0_4[2] - DH[0][1]) ** 2 + matrix0_4[1] ** 2 + matrix0_4[0] ** 2)
    #p14_1 = matrix0_4 - np.array([330 * np.cos(sp[0]), 330 * np.sin(sp[0]), 0])
    #p14_2 = matrix0_4 - np.array([330 * np.cos(sp[1]), 330 * np.sin(sp[1]), 0])

    #theta_3_1 = np.arccos((1.15 ** 2 + 1.225 ** 2 - p14_1 ** 2) / 2 * 1.15 * 1.225)
    #theta_3_2 = np.arccos((1.15 ** 2 + 1.225 ** 2 - p14_2 ** 2) / 2 * 1.15 * 1.225)
    theta_3_1 = np.degrees(np.arccos(-(DH[1][2] ** 2 + DH[3][1] ** 2 - np.abs(o1o4) ** 2)/ 2 * DH[1][2] * DH[3][1]))
    theta_3_2 = np.degrees(- np.arccos(-(DH[1][2] ** 2 + DH[3][1] ** 2 - np.abs(o1o4) ** 2) / 2 * DH[1][2] * DH[3][1]))
    print(theta_3_1, theta_3_2)


joints = np.array([0, 0, 0, 0, 0, 0])
fk(joints)


calculating_theta1(fk(joints), dh_parameters(joints))
calculating_theta2(fk(joints), dh_parameters(joints), calculating_theta1(fk(joints), dh_parameters(joints)))
