import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    config = os.path.join(
        get_package_share_directory('mixer_wrapper'),
        'config',
        'mixer_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mixer_wrapper',
            executable='mixer_com',
            name='MIXER_WRAPPER',
            output='screen',
            parameters=[config],
            # Add remappings if needed
            # remappings=[
            #     ('/mixer/telemetry/gps', '/gps/data'),
            # ]
        )
    ])
