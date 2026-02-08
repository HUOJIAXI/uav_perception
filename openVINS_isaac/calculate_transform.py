#!/usr/bin/env python3
"""Calculate correct T_imu_cam from Isaac Sim camera setup"""
import numpy as np

# From launch_with_camera.py line 146:
# camera_rot = R.from_euler('xyz', [-90, 0, 90], degrees=True)

# Manually compute rotation matrix for [-90, 0, 90] degrees XYZ Euler
def euler_to_matrix(roll, pitch, yaw):
    """Convert XYZ Euler angles (degrees) to rotation matrix"""
    # Convert to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Rotation matrices (extrinsic XYZ order)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    # Extrinsic XYZ: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R

# Camera Euler angles from Isaac Sim
roll_x = -90.0   # Pitch down
pitch_y = 0.0    # No roll
yaw_z = 90.0     # Yaw left

R_cam = euler_to_matrix(roll_x, pitch_y, yaw_z)

print("Camera Rotation Matrix (body to camera):")
print(R_cam)
print()

# Camera position from line 153: [0.1, 0.0, 0.05]
t_cam = np.array([0.1, 0.0, 0.05])

print("Camera Position (meters):")
print(f"  Forward: {t_cam[0]}")
print(f"  Right:   {t_cam[1]}")
print(f"  Up:      {t_cam[2]}")
print()

# Build T_imu_cam (4x4 homogeneous transform)
T_imu_cam = np.eye(4)
T_imu_cam[:3, :3] = R_cam
T_imu_cam[:3, 3] = t_cam

print("T_imu_cam (4x4):")
print(T_imu_cam)
print()

print("="*70)
print("YAML format for kalibr_imucam_chain.yaml:")
print("="*70)
print("  T_imu_cam:")
for i in range(4):
    row = T_imu_cam[i, :]
    print(f"    - [{row[0]:.10f}, {row[1]:.10f}, {row[2]:.10f}, {row[3]:.10f}]")
print()

# Simplify (remove numerical errors)
print("Simplified (removing numerical noise):")
print("  T_imu_cam:")
T_simple = np.round(T_imu_cam, decimals=10)
for i in range(4):
    row = T_simple[i, :]
    print(f"    - [{row[0]}, {row[1]}, {row[2]}, {row[3]}]")
