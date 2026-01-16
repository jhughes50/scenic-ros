"""
    @Author Jason Hughes
    @Date January 2025 

    @About launch scenic is a ROS 2 node
"""

import os 
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path to parameter files
    ros_params_file = "/usr/local/share/scenic/config/ros-params.yaml" 
    graph_params_file = "/usr/local/share/scenic/config/glider-params.yaml"

    # create logging directory
    with open(graph_params_file) as f:
        config = yaml.safe_load(f)
    os.makedirs(config["logging"]["directory"], exist_ok=True)

    node = Node(package="scenic_ros",
                executable="scenic_ros_node",
                name="scenic_node",
                output="screen",
                parameters=[
                    ros_params_file,
                    {"glider_path": graph_params_file,
                     "use_sim_time": use_sim_time,
                     "use_odom": False}
                ],
                remappings=[
                    ('/gps', '/ublox_gps_node/fix'),
                    ('/imu', '/vectornav/imu'),
                    ('/image', '/cam_driver/image_raw')
                ]
                )

    return LaunchDescription([use_sim_time_arg, node])
