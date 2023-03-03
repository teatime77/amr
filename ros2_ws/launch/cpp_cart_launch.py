import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('cpp_cart')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(share_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        Node(
            package='cpp_cart',
            executable='talker',
        ),
        Node(
            package='cart_gui',
            executable='talker',
        ),
        tf2_node,
        robot_localization_node,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        )
    ])
