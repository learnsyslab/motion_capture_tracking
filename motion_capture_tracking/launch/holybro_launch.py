import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# MAVROS runs on the companion computer of the vehicle, so the poses have to reach beyond this
# machine. Unlike the other launch files, the discovery range must not be pinned to LOCALHOST,
# which would keep the vision estimate from ever arriving.
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"


def generate_launch_description():
    node_config = os.path.join(
        get_package_share_directory("motion_capture_tracking"),
        "config",
        "holybro_cfg.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="motion_capture_tracking",
                executable="motion_capture_tracking_node",
                name="motion_capture_tracking",
                output="screen",
                parameters=[node_config],
            ),
        ]
    )
