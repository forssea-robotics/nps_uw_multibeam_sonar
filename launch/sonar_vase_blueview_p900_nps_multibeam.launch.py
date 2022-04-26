#! /usr/bin/env python3

import os
import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    
    
    
    # pkg_share = launch_ros.substitutions.FindPackageShare(package='robochon_description').find('robochon_description')
    # default_model_path = os.path.join(pkg_share, 'urdf/robochon.urdf.xacro')
    
    

    
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )

    # spawn_entity = launch_ros.actions.Node(
    # 	package='gazebo_ros', 
    # 	executable='spawn_entity.py',
    #     arguments=['-entity', 'robochon', '-topic', 'robot_description',"-y","3"],
    #     output='screen'
    # )
    
    
    
    
    
  
    # Using the standard Gazebo launch file
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='nps_uw_multibeam_sonar').find('nps_uw_multibeam_sonar')
    default_model_path = os.path.join(pkg_share, 'urdf/multibeam_sonar_blueview_p900.xacro')

    # Gazebo Server
    gzserver_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    world_file = os.path.join(get_package_share_directory(
        'nps_uw_multibeam_sonar'), 'worlds', 'sonar_vase_blueview_p900_nps_multibeam.world')
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file), launch_arguments={
            'world': world_file, 'verbose': 'verbose'}.items())

    # Gazebo Client
    gzclient_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    gzclient_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gzclient_launch_file))
    
    sonar_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'),
                                                  #  ' standalone:=', LaunchConfiguration('standalone'),
                                                  #  ' name:=', LaunchConfiguration('sonar_name'),
                                                  #  ' scale:=', LaunchConfiguration('sonar_scale'),
                                                  #  ' debug:=', LaunchConfiguration('debug'),
                                                  #  ' sonar_image_topic:=', LaunchConfiguration('sonar_image_topic'),
                                                  #  ' sonar_image_raw_topic:=', LaunchConfiguration('sonar_image_raw_topic'),
                                                  #  ' maxDistance:=', LaunchConfiguration('maxDistance'),
                                                  #  ' raySkips:=', LaunchConfiguration('raySkips'),
                                                  #  ' sensorGain:=', LaunchConfiguration('sensorGain'),
                                                  #  ' plotScaler:=', LaunchConfiguration('plotScaler'),
                                                  #  ' writeLog:=', LaunchConfiguration('writeLog'),
                                                  #  ' writeFrameInterval:=', LaunchConfiguration('writeFrameInterval'),
                                                  #  ' artificialVehicleVibration:=', LaunchConfiguration('artificialVehicleVibration'),
                                                  #  ' constantReflectivity:=', LaunchConfiguration('constantReflectivity'),
                                                  #  ' reflectivityDatabaseFile:=', LaunchConfiguration('reflectivityDatabaseFile'),
                                                  #  ' ray_visual:=', LaunchConfiguration('ray_visual'),
                                                   ])}]
    )
    
    # Spawn sonar model
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
      arguments=["-entity", 'sonar_name', '-topic', 'robot_description', '-urdf', '-x', 'sonar_x', "-y", 'sonar_y', "-z", 'sonar_z', "-R", 'sonar_roll', "-P", 'sonar_pitch', "-Y", 'sonar_yaw'],
      output='screen'
    )
    
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
        arguments=["--ros-args --remap image:=/sonar_image"],
        parameters=[{"window_name": "seabat_f50", "autosize": True, "filename_format": "/tmp/SonarImage_capture_%04i.jpg"}],
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
        DeclareLaunchArgument(name='model', default_value=default_model_path),
        DeclareLaunchArgument(name='verbose', default_value="true"),
        DeclareLaunchArgument(name='standalone', default_value="true"),
        DeclareLaunchArgument(name='sonar_name', default_value="blueview_p900"),
        DeclareLaunchArgument(name='sonar_x', default_value="0"),
        DeclareLaunchArgument(name='sonar_y', default_value="2"),
        DeclareLaunchArgument(name='sonar_z', default_value="3"),
        DeclareLaunchArgument(name='sonar_roll', default_value="0"),
        DeclareLaunchArgument(name='sonar_pitch', default_value="0"),
        DeclareLaunchArgument(name='sonar_yaw', default_value="-1.5708"),
        DeclareLaunchArgument(name='maxDistance', default_value="4"),
        DeclareLaunchArgument(name='raySkips', default_value="1"),
        DeclareLaunchArgument(name='sonar_scale', default_value="'1 1 1'"),
        DeclareLaunchArgument(name='sonar_image_topic', default_value="sonar_image"),
        DeclareLaunchArgument(name='sonar_image_raw_topic', default_value="sonar_image_raw"),
        DeclareLaunchArgument(name='ray_visual', default_value="true"),
        DeclareLaunchArgument(name='plotScaler', default_value="0"),
        DeclareLaunchArgument(name='sensorGain', default_value="0.04"),
        DeclareLaunchArgument(name='writeLog', default_value="false"),
        DeclareLaunchArgument(name='debug', default_value="false"),
        DeclareLaunchArgument(name='writeFrameInterval', default_value="5"),
        DeclareLaunchArgument(name='artificialVehicleVibration', default_value="false"),
        DeclareLaunchArgument(name='constantReflectivity', default_value="true"),
        DeclareLaunchArgument(name='reflectivityDatabaseFile', default_value="variationalReflectivityDatabase.csv"),
        gzserver_launch, 
        gzclient_launch, 
        sonar_state_publisher_node,
        spawn_entity,
        static_tf_node,
        image_view_sonar_node
        ])
