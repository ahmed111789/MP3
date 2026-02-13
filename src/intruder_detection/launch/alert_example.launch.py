import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'intruder_detection'
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'alert.yaml'
    )
    print(f"Loading config from: {config_file}")

    # 2. Define the node
    detector_node = Node(
        package=pkg_name,
        executable='intruder_detection', 
        name='human_detector',       # Overwrites the node name (optional)
        output='screen',
        emulate_tty=True,            # Better color logging in terminal
        parameters=[config_file]     # Load the YAML
    )
    camera_node = Node(
        package="usb_cam",
        executable='usb_cam_node_exe'
    )

    return LaunchDescription([
        camera_node,
        detector_node
        
    ])
