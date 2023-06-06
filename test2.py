#%matplotlib widget

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import numpy as np
np.set_printoptions(linewidth=100, suppress=True)

# Define link properties
link_lengths = [0.25, 0.25, 0.25, 0.10, 0]
density = 2700  # alu density kg/m^3

g = 9.81

# Calculate size and mass of each link
link_sizes = [(l, l/10/2) for l in link_lengths]
link_masses = []
for size in link_sizes:
    volume = np.pi * (size[1] ** 2) * size[0]
    mass = density * volume
    link_masses.append(mass)

# Centre of mass - assume it's in the middle of each link for simplicity
link_centers = [l / 2 for l, _ in link_sizes]

qlim1 = np.deg2rad([-80, 80])
qlim2 = np.deg2rad([-90, 90])
qlim3 = np.deg2rad([-90, 90])
qlim4 = np.deg2rad([-90, 90])
qlim5 = np.deg2rad([-90, 90])

# Define the robot links
links = [
    RevoluteDH(alpha=pi/2, d=link_lengths[0], qlim=qlim1, m=link_masses[0], r=[0, 0, link_centers[0]]),
    RevoluteDH(a=link_lengths[1], qlim=qlim2, m=link_masses[1], r=[0, 0, link_centers[1]]),
    RevoluteDH(a=link_lengths[2], qlim=qlim3, m=link_masses[2], r=[0, 0, link_centers[2]]),
    RevoluteDH(a=link_lengths[3], qlim=qlim4, m=link_masses[3], r=[0, 0, link_centers[3]]),
    RevoluteDH(alpha=-pi/2, qlim=qlim5, m=link_masses[4], r=[0, 0, link_centers[4]])
]

# Create the robot
robot = DHRobot(links, gravity=[0, g, 0])

print(robot)

#%matplotlib inline
angles = [0, 45, -45, -90, 90]

#convert deg to rad angles
angles = np.deg2rad(angles)


robot.plot(angles, backend='pyplot', block=True)
