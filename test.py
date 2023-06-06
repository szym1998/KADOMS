#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi


def main():
    np.set_printoptions(precision=3, suppress=True)

    dh_params = np.array([[0.25, 0., -pi / 2, 0.],
                          [0., 0., pi / 2, 0.],
                          [0.25, 0., -pi / 2, 0.],
                          [0., 0., pi / 2, 0.],
                          [0.25, 0., -pi / 2, 0.],
                          [0., 0., pi / 2, 0.],
                          [0.25, 0., 0., 0.]])
    robot = RobotSerial(dh_params)

    # =====================================
    # forward
    # =====================================

    theta = np.array([45,90,45,0,0,0,0])

    #convert angles in degrees to radians from theta 
    theta = np.deg2rad(theta)
    f = robot.forward(theta)

    print("-------forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    print("end frame xyz:")
    print(f.t_3_1.reshape([3, ]))
    print("end frame abc:")
    print(f.euler_3)
    print("end frame rotational matrix:")
    print(f.r_3_3)
    print("end frame quaternion:")
    print(f.q_4)
    print("end frame angle-axis:")
    print(f.r_3)

    robot.show()


if __name__ == "__main__":
    main()