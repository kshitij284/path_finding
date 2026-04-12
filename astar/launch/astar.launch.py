from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='17'),
        DeclareLaunchArgument('goal_y', default_value='19'),
        DeclareLaunchArgument('allow_diagonal', default_value='false'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', './astar/rviz/config.rviz']
        ),
        Node(
            package='astar',
            executable='astar_node',
            name='astar',
            parameters=[{'goal_x': LaunchConfiguration('goal_x'),
                         'goal_y': LaunchConfiguration('goal_y'),
                         'allow_diagonal': LaunchConfiguration('allow_diagonal')}]
        )
    ])



