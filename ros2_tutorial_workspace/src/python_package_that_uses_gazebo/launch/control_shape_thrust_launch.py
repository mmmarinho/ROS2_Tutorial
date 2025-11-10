from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    node = Node(
            output='screen',
            emulate_tty=True,
            package='python_package_that_uses_gazebo',
            executable='control_shape_thurst_node',
            name='control_shape_thurst_node'
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config_bridge', 'control_shape_thurst.yaml')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        node,
        bridge
    ])
