#! /usr/bin/env python3

import os
import launch_ros
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  
    # Using the standard Gazebo launch file
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='nps_uw_multibeam_sonar').find('nps_uw_multibeam_sonar')
    default_model_path = os.path.join(pkg_share, 'urdf/multibeam_sonar_blueview_p900.xacro')

    # Gazebo Server
    gzserver_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    world_file = os.path.join(get_package_share_directory(
        'nps_uw_multibeam_sonar'), 'worlds', 'sonar_tank.world')
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file), launch_arguments={
            'world': world_file, 'verbose': 'verbose'}.items())

    # Gazebo Client
    gzclient_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    gzclient_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gzclient_launch_file))
    
    # Retrieve sonar parameters
    config_file = os.path.join(pkg_share, 'config/blueview_p900_tank.yaml')
    with open(config_file, mode='r') as file:
        params = yaml.full_load(file)
        
    # Generate sonar model
    doc = xacro.process_file(default_model_path, mappings={'standalone': params['standalone'],
                                                            'name': params['sonar_name'],
                                                            'gpu_ray': params['gpu_ray'],
                                                            'scale': params['sonar_scale'],
                                                            'debug': params['debug'],
                                                            'sonar_image_topic': params['sonar_image_topic'],
                                                            'sonar_image_raw_topic': params['sonar_image_raw_topic'],
                                                            'maxDistance': params['maxDistance'],
                                                            'raySkips': params['raySkips'],
                                                            'sensorGain': params['sensorGain'],
                                                            'plotScaler': params['plotScaler'],
                                                            'writeLog': params['writeLog'],
                                                            'writeFrameInterval': params['writeFrameInterval'],
                                                            'ray_visual': params['ray_visual']
                                                           })
    robot_desc = doc.toprettyxml(indent='  ')
    sonar_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}])
    
    # Spawn sonar model
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
      arguments=["-entity", params['sonar_name'], '-topic', 'robot_description', '-x', params['sonar_x'], "-y", params['sonar_y'], "-z", params['sonar_z'], "-R", params['sonar_roll'], "-P", params['sonar_pitch'], "-Y", params['sonar_yaw']],
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
        remappings=[
                 ('image', params['sonar_image_topic']),
             ],
        parameters=[{"window_name": params['sonar_name'], "autosize": True, "filename_format": "/tmp/SonarImage_capture_%04i.jpg"}],
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
        gzserver_launch, 
        gzclient_launch, 
        sonar_state_publisher_node,
        spawn_entity,
        static_tf_node,
        image_view_sonar_node
        ])
