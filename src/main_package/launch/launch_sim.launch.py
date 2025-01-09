import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def generate_launch_description():

    #  variables
    package_name = 'main_package'
    rviz_name = 'rviz_sim.rviz'

    # launch args
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_rviz_launch_arg = DeclareLaunchArgument(
        name='rviz',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config',
                                   rviz_name),
        description='Full path to .rviz file')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,
                              'description',
                              'robot',
                              'robot.urdf.xacro')

    robot_description_config = Command(
        ['xacro ',
         xacro_file,
         ' sim_mode:=',
         use_sim_time])

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time,
    }

    node_robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='screen',
                                      parameters=[params])

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    [
                        os.path.join(get_package_share_directory(package_name),
                                     'config',
                                     rviz_name)
                    ]
                ])

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])

    realsense = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("realsense2_camera")),
        '/launch',
        '/rs_launch.py'
    ]),
                                         launch_arguments={
                                             'pointcloud.enable': 'true',
                                         }.items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use sim time if true'),
        DeclareLaunchArgument('publish_rate',
                              default_value='1000.0',
                              description='Publish rate'), # Bridge
        declare_rviz_launch_arg,
        node_robot_state_publisher,
        rviz,
        realsense,
    ])