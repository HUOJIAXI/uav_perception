#!/usr/bin/env python3
"""Calculate inverse of T_imu_cam"""
import numpy as np

# Current T_imu_cam (might be backwards!)
T_current = np.array([
    [0.0, 0.0, -1.0, 0.1],
    [1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.05],
    [0.0, 0.0, 0.0, 1.0]
])

print("Current T_imu_cam:")
print(T_current)
print()

# Compute inverse
T_inverse = np.linalg.inv(T_current)

print("Inverse T_imu_cam:")
print(T_inverse)
print()

print("YAML format:")
print("  T_imu_cam:")
for i in range(4):
    row = T_inverse[i, :]
    print(f"    - [{row[0]:.1f}, {row[1]:.1f}, {row[2]:.1f}, {row[3]:.10f}]")
