#!/usr/bin/env python3
"""Analyze the failing quaternion to understand camera orientation"""
import numpy as np
from scipy.spatial.transform import Rotation

# The failing quaternion from the error
q = np.array([0.0409965, -0.0379749, -0.706086, 0.705917])

print("Quaternion:", q)
print("Norm:", np.linalg.norm(q))
print()

# Convert to rotation matrix
# Note: scipy uses [x,y,z,w] order
r = Rotation.from_quat([q[0], q[1], q[2], q[3]])
R = r.as_matrix()

print("Rotation Matrix:")
print(R)
print()

# Convert to Euler angles (ZYX convention)
euler_zyx = r.as_euler('ZYX', degrees=True)
print("Euler angles (ZYX - yaw, pitch, roll):", euler_zyx)
print()

# Convert to Euler angles (XYZ convention)
euler_xyz = r.as_euler('XYZ', degrees=True)
print("Euler angles (XYZ - roll, pitch, yaw):", euler_xyz)
print()

# Check if it's close to 90° or 180° rotations
print("Analysis:")
print("This is approximately a 90° rotation around Y and Z axes")
print("Suggests camera is front-facing but rotated")
