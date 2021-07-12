import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    default_model_path = os.path.join(pkg_share, 'src/description/rover.urdf')
    package_dir = get_package_share_directory('rover_webots')
    core_dir = get_package_share_directory('webots_ros2_core')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'rover_webots'),
            ('executable', 'slave'),
            ('world', PathJoinSubstitution(
                [package_dir, 'worlds', 'Prueba de Visual.wbt'])),
        ]
    )

    master = Node(
        package='rover_webots',
        executable='master',
        name='master_node'
    )   

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        webots,
    ])
