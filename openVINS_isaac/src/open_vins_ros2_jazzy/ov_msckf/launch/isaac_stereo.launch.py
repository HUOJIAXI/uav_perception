"""
Isaac Sim Stereo Camera + OpenVINS Launch File

Publishes the TF tree for the Isaac Sim UAV stereo camera setup and
launches OpenVINS in stereo mode.

TF Tree:
  base_link -> imu_link          (identity, IMU at body center)
  base_link -> stereo_left_optical   (left camera optical frame)
  base_link -> stereo_right_optical  (right camera optical frame)

Isaac Sim Topics:
  /drone0/stereo_left/color/image_raw
  /drone0/stereo_right/color/image_raw
  /telemetry/imu
  /clock

Usage:
  ros2 launch ov_msckf isaac_stereo.launch.py
  ros2 launch ov_msckf isaac_stereo.launch.py rviz_enable:=true
  ros2 launch ov_msckf isaac_stereo.launch.py verbosity:=DEBUG
"""

import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# --------------------------------------------------------------------------
# Stereo camera geometry (must match Isaac Sim launch_with_stereo_camera.py)
# --------------------------------------------------------------------------
STEREO_BASELINE = 0.055       # 55 mm
CAMERA_FORWARD  = 0.10        # 10 cm forward from body centre
CAMERA_UP       = 0.05        # 5 cm above body centre
HALF_BASELINE   = STEREO_BASELINE / 2.0

# Camera optical frame orientation relative to body (FLU)
# Optical: X=right, Y=down, Z=forward
# Body:    X=forward, Y=left, Z=up
#
# As quaternion (x, y, z, w) for the rotation from body to optical frame:
#   R_body_to_optical = R_CtoI^(-1) = R_CtoI^T
#   R_CtoI = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
#   R_ItoC = R_CtoI^T = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
#
# This rotation as quaternion (x, y, z, w):
#   Rotation: body_X -> optical_Z, body_Y -> -optical_X, body_Z -> -optical_Y
#   qx=0.5, qy=-0.5, qz=0.5, qw=0.5
CAM_QUAT_X = 0.5
CAM_QUAT_Y = -0.5
CAM_QUAT_Z = 0.5
CAM_QUAT_W = 0.5


launch_args = [
    DeclareLaunchArgument(
        name="namespace", default_value="ov_msckf",
        description="namespace for OpenVINS nodes",
    ),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true",
        description="enable OpenVINS node",
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false",
        description="enable rviz2 node",
    ),
    DeclareLaunchArgument(
        name="repub_enable", default_value="true",
        description="enable odom_republisher node",
    ),
    DeclareLaunchArgument(
        name="verbosity", default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_sim_time", default_value="true",
        description="use simulation time from /clock topic",
    ),
]


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    # ---- Config path ----
    config_path = os.path.join(
        get_package_share_directory("ov_msckf"),
        "config", "isaac_stereo", "estimator_config.yaml",
    )
    if not os.path.isfile(config_path):
        return [
            LogInfo(msg="ERROR: config not found: '{}' - rebuild the workspace first.".format(config_path))
        ]

    # ---- Static TF: base_link -> stereo_left_optical ----
    tf_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_stereo_left",
        arguments=[
            "--x",  str(CAMERA_FORWARD),
            "--y",  str(HALF_BASELINE),
            "--z",  str(CAMERA_UP),
            "--qx", str(CAM_QUAT_X),
            "--qy", str(CAM_QUAT_Y),
            "--qz", str(CAM_QUAT_Z),
            "--qw", str(CAM_QUAT_W),
            "--frame-id",       "base_link",
            "--child-frame-id", "stereo_left_optical",
        ],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
    )

    # ---- Static TF: base_link -> stereo_right_optical ----
    tf_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_stereo_right",
        arguments=[
            "--x",  str(CAMERA_FORWARD),
            "--y",  str(-HALF_BASELINE),
            "--z",  str(CAMERA_UP),
            "--qx", str(CAM_QUAT_X),
            "--qy", str(CAM_QUAT_Y),
            "--qz", str(CAM_QUAT_Z),
            "--qw", str(CAM_QUAT_W),
            "--frame-id",       "base_link",
            "--child-frame-id", "stereo_right_optical",
        ],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
    )

    # ---- Static TF: base_link -> imu_link (identity) ----
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu",
        arguments=[
            "--x",  "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id",       "base_link",
            "--child-frame-id", "imu_link",
        ],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
    )

    # ---- OpenVINS node ----
    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": True},
            {"max_cameras": 2},
            {"save_total_state": False},
            {"config_path": config_path},
            {"use_sim_time": use_sim_time == "true"},
        ],
    )

    # ---- RViz2 node ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d" + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args", "--log-level", "warn",
        ],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
    )

    # ---- Odom republisher node ----
    repub_node = Node(
        package="ov_repub",
        executable="odom_republisher",
        condition=IfCondition(LaunchConfiguration("repub_enable")),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time == "true"}],
    )

    return [
        tf_left,
        tf_right,
        tf_imu,
        openvins_node,
        rviz_node,
        repub_node,
    ]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
