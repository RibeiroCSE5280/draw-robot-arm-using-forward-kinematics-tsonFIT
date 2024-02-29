#!/usr/bin/env python
# coding: utf-8


from vedo import *
import matplotlib as plt


def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)

    if axis_name == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, c, -s],
                                    [0, s, c]])
    if axis_name == 'y':
        rotation_matrix = np.array([[c, 0, s],
                                    [0, 1, 0],
                                    [-s, 0, c]])
    elif axis_name == 'z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s, c, 0],
                                    [0, 0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1

    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)

    originDot = Sphere(pos=[0, 0, 0],
                       c="black",
                       r=0.10)

    # Combine the axes together to form a frame as a single mesh object
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot

    return F


def getLocalFrameMatrix(R_ij, t_ij):
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij, t_ij],
                     [np.zeros((1, 3)), 1]])

    return T_ij


def forward_kinematics(p1, l1, l2, l3, l4, phi):
    frames = []

    R_01 = RotationMatrix(phi[0], axis_name='z')
    t_01 = p1
    T_01 = getLocalFrameMatrix(R_01, t_01)

    frame_one_arrows = createCoordinateFrameMesh()
    link_one_mesh = Cylinder(r=1,
                             height=l1,
                             pos=(0, 0, l1 / 2),
                             c="yellow",
                             alpha=.8,
                             axis=(0, 0, 1)
                             )
    frame_one = frame_one_arrows + link_one_mesh
    frame_one.apply_transform(T_01)
    frames.append(frame_one)

    R_12 = RotationMatrix(phi[1], axis_name='y')
    t_12 = np.array([[0.0], [0.0], [l1 + 0.4]])
    T_12 = getLocalFrameMatrix(R_12, t_12)

    T_02 = T_01 @ T_12

    frame_two_arrows = createCoordinateFrameMesh()
    link_two_mesh = Cylinder(r=0.4,
                             height=l2,
                             pos=(0, 0, l2 / 2 + 0.4),
                             c="red",
                             alpha=.8,
                             axis=(0, 0, 1)
                             )
    r = 0.4
    sphere = Sphere(r=r).pos(0, 0, 0).color("grey").alpha(.8)
    frame_two = frame_two_arrows + link_two_mesh + sphere
    frame_two.apply_transform(T_02)
    frames.append(frame_two)

    R_23 = RotationMatrix(phi[2], axis_name='y')
    t_23 = np.array([[0.0], [0.0], [l2 + 0.4]])
    T_23 = getLocalFrameMatrix(R_23, t_23)

    T_03 = T_02 @ T_23

    frame_three_arrows = createCoordinateFrameMesh()
    link_three_mesh = Cylinder(r=0.4,
                               height=l3,
                               pos=(0, 0, l3 / 2 + 0.4),
                               c="orange",
                               alpha=.8,
                               axis=(0, 0, 1)
                               )
    frame_three = frame_three_arrows + link_three_mesh + sphere
    frame_three.apply_transform(T_03)
    frames.append(frame_three)

    R_34 = RotationMatrix(phi[3], axis_name='y')
    t_34 = np.array([[0.0], [0.0], [l3 + 0.4]])
    T_34 = getLocalFrameMatrix(R_34, t_34)

    T_04 = T_03 @ T_34

    frame_four_arrows = createCoordinateFrameMesh()
    sphere = Sphere(r=0).pos(0, 0, 0).color("grey").alpha(.8)
    frame_four = frame_four_arrows + sphere
    frame_four.apply_transform(T_04)
    frames.append(frame_four)

    return frames


def plot_frames(frames):
    base = createCoordinateFrameMesh()
    frames.insert(0, base)

    plt = Plotter()
    axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))
    plt.show(frames, axes, viewup="z")


def animate_frames(p1, l1, l2, l3, l4, phi):
    for i in range(0, 10, 5):
        phis = [phi[0], i, 0, 0]
        frames = forward_kinematics(p1, l1, l2, l3, l4, phis)
        base = createCoordinateFrameMesh()
        frames.insert(0, base)
        plt = Plotter(offscreen=True)
        axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))
        plt.show(frames, axes, viewup=[1, 1, 1])
        screenshot('screen_i_%02d.png' % i)

    for i in range(0, 30, 5):
        phis = [phi[0], 10, i, 0]
        frames = forward_kinematics(p1, l1, l2, l3, l4, phis)
        base = createCoordinateFrameMesh()
        frames.insert(0, base)
        plt = Plotter(offscreen=True)
        axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))
        plt.show(frames, axes, viewup=[1, 1, 1])
        screenshot('screen_i_%02d.png' % i)

    for i in range(0, 50, 5):
        phis = [phi[0], 10, 30, i*5]
        frames = forward_kinematics(p1, l1, l2, l3, l4, phis)
        base = createCoordinateFrameMesh()
        frames.insert(0, base)
        plt = Plotter(offscreen=True)
        axes = Axes(xrange=(-10, 10), yrange=(-10, 10), zrange=(0, 10))
        plt.show(frames, axes, viewup=[1, 1, 1])
        screenshot('screen_i_%02d.png' % i)


def main():
    p1 = np.array([[3.0], [2.0], [0.0]])
    l1, l2, l3, l4 = [5, 8, 3, 4]
    phi = [120, 50, 60, 45]

    # frames = forward_kinematics(p1, l1, l2, l3, l4, phi)
    # plot_frames(frames)

    animate_frames(p1, l1, l2, l3, l4, phi)


if __name__ == '__main__':
    main()
