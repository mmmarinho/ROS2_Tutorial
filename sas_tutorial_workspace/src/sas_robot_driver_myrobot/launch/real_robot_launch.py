"""
Run this script in a different terminal window or tab. Be ready to close this,
as this activates the real robot if the connection is successful.
"""
import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_myrobot',
            executable='sas_robot_driver_myrobot_node',
            name='myrobot_1',
            parameters=[{
                "ip": "127.0.0.1",
                "joint_limits_min": [-360.0, -360.0, -360.0, -360.0, -360.0, -720.0],
                "joint_limits_max": [360.0, 360.0, 360.0, 360.0, 360.0, 720.0],
                "thread_sampling_time_sec": 0.002 # Robot thread is at 500 Hz
            }]
        ),

    ])