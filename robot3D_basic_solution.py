from robot3D_basic import *


def forward_kinematics(phi, l1, l2, l3, l4):
    R_z_1 = RotationMatrix(phi[0], 'z')
    R_z_2 = RotationMatrix(phi[1], 'z')
    R_z_3 = RotationMatrix(phi[2], 'z')
    R_z_4 = RotationMatrix(phi[3], 'z')

    p1 = np.array([[3.0], [2.0], [0.0]])
    t_x_1 = np.array([[l1+2*0.4], [0.0], [0.0]])
    t_x_2 = np.array([[l2+2*0.4], [0.0], [0.0]])
    t_x_3 = np.array([[l3+0.4], [0.0], [0.0]])

    T_01 = getLocalFrameMatrix(R_z_1, p1)
    T_12 = getLocalFrameMatrix(R_z_2, t_x_1)
    T_23 = getLocalFrameMatrix(R_z_3, t_x_2)
    T_34 = getLocalFrameMatrix(R_z_4, t_x_3)

    T_02 = T_01 @ T_12
    T_03 = T_02 @ T_23
    T_04 = T_03 @ T_34

    e = T_04[:3, 3]

    return T_01, T_02, T_03, T_04, e
