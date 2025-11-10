from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    send_poses = Node(
            output='screen',
            emulate_tty=True,
            package='python_package_that_uses_gazebo',
            executable='send_poses_to_gazebo_node',
            name='send_poses_to_gazebo_node'
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/shapes_with_tf2_and_wrench/set_pose@roz_gz_interfaces/srv/EntityPose'],
        output='screen'
    )
    
    return LaunchDescription([
        send_poses,
        bridge
    ])
