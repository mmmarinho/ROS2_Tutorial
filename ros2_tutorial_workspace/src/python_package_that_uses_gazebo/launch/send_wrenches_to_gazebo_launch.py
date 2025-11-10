from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    send_wrenches = Node(
            output='screen',
            emulate_tty=True,
            package='python_package_that_uses_gazebo',
            executable='send_wrenches_to_gazebo_node',
            name='send_wrenches_to_gazebo_node'
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/shapes_with_tf2_and_wrench/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench'],
        output='screen'
    )
    
    return LaunchDescription([
        send_wrenches,
        bridge
    ])
