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

    name = "leader"

    return LaunchDescription([
        Node(
            package='mixer_wrapper',
            executable='mixer_com',
            name='MIXER_WRAPPER',
            output='screen',
            parameters=[config],
            # Add remappings if needed
            remappings=[
                ('/mixer/telemetry/rtk', '/'+name+'/telemetry/rtk'),
                ('/mixer/telemetry/gps', '/'+name+'/telemetry/gps'),
                ('/mixer/enable_calibration', '/'+name+'/enable_calibration'),
                ('/mixer/telemetry/barometer','/'+name+'/telemetry/barometer'),
                ('/mixer/telemetry/imu','/'+name+'/telemetry/imu'),
                ('/mixer/telemetry/magnetometer','/'+name+'/telemetry/magnetometer'),
            ]
        )
    ])