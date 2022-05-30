#! /usr/bin/env python3

from ast import arguments
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Using the standard Gazebo launch file
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo Server
    gzserver_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    world_file = os.path.join(get_package_share_directory(
        'nps_uw_multibeam_sonar'), 'worlds', 'local_search_blueview_p900_nps_multibeam_raster.world')
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file), launch_arguments={
            'world': world_file, 'verbose': 'true'}.items())

    # Gazebo Client
    gzclient_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    gzclient_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gzclient_launch_file))
    
    # Static transform publisher
    static_tf_node = Node(
        package="tf2_ros",
        arguments=["0", "0", "0", "0", "0", "0", "map", "forward_sonar_optical_link"],
        executable="static_transform_publisher",
        name="multibeam_sonar_base_link"
    )

    # Image viewer from sonar image
    image_view_sonar_node = Node(
        package="image_view",
        remappings=[
                 ('image', '/sonar_image'),
             ],
        parameters=[{"window_name": "blueview_p900", "autosize": True, "filename_format": "/tmp/SonarImage_capture_%04i.jpg"}],
        executable="image_view",
        output="screen",
        name="image_view_sonar"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name="gui", default_value="true"),
        DeclareLaunchArgument(name="paused", default_value="false"),
        DeclareLaunchArgument(name="manager", default_value="true"),
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        gzserver_launch, 
        gzclient_launch, 
        static_tf_node,
        image_view_sonar_node
        ])
