import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('cpp_cart')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )

    return LaunchDescription([
        Node(
            package='cpp_cart',
            executable='talker',
        ),
        Node(
            package='cart_gui',
            executable='talker',
        ),
        tf2_node,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        )
    ])
