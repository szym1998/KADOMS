from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
from math import pi
import numpy as np
import open3d as o3d

def main():
    np.set_printoptions(precision=3, suppress=True)
    dh_params = np.array([[0.25, 0, np.pi/2, 0],
                          [0, 0.25, 0, 0],
                          [0, 0.25, 0, 0],
                          [0, 0.1, 0, 0],
                          [0, 0, -np.pi/2, np.pi/2]])

    robot = RobotSerial(dh_params)
    # =====================================
    # forward
    # =====================================

    # Initialize an empty list to store end frame positions
    positions = []

    # Create range of angles based on joint limitations
    angle_range_q1 = np.arange(-80, 80, 5)
    angle_range_q2_q5 = np.arange(-90, 90, 20)

    for a in angle_range_q1:
        for b in angle_range_q2_q5:
            for c in angle_range_q2_q5:
                for d in angle_range_q2_q5:
                    for e in angle_range_q2_q5:
                        theta = np.array([a, b, c, d, e])
                        theta = np.deg2rad(theta)
                        f = robot.forward(theta)

                        # Append end frame position to positions list
                        positions.append(f.t_3_1.reshape([3, ]))

    positions = np.array(positions)  # Convert list to numpy array

    # Create open3d point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)

    # Save the point cloud to a file
    o3d.io.write_point_cloud("points.pcd", pcd)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

    #robot.show()

if __name__ == "__main__":
    main()

