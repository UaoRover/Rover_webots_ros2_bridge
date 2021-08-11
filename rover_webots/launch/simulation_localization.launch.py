import launch
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.substitutions.path_join_substitution import PathJoinSubstitution
import launch.actions
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import launch_ros
import launch_ros.actions
import launch_ros.descriptions
import os
import yaml
import pathlib
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    default_model_path = os.path.join(pkg_share, 'src/description/rover.urdf')
    package_dir = get_package_share_directory('rover_webots')
    core_dir = get_package_share_directory('webots_ros2_core')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    parameters_file_dir = os.path.join(package_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)


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
            ('executable', 'main'),
            ('publish_tf','False'),
            ('world', PathJoinSubstitution(
                [package_dir, 'worlds', 'entorno_real.wbt'])),
        ]
    )
    ekf_local=launch_ros.actions.Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/local')]           
    )
    ekf_global=launch_ros.actions.Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_map',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/global')]
    )
    nav_sat=launch_ros.actions.Node(
        package='robot_localization', 
        executable='navsat_transform_node', 
        name='navsat_transform',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('imu/data', 'imu'),
                    ('gps/fix', 'gps'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')]           
    ) 

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
        ekf_local,
        ekf_global,
	])
