import launch
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import ament_index_python.packages
import sys

def generate_launch_description():

    # print(sys.argv[0])
    # print(__file__)
    print("****************")

    print(get_package_share_directory('carla_shenlan_mpc_controller'))
    print(os.getcwd())
    
    # param参数配置文件路径
    lqr_parameters_configuration = os.path.join(
        os.getcwd(), 
        'src/ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/config', 
        'mpc_parameters_configuration.yaml'
    )
    print(lqr_parameters_configuration)

    rviz_config_dir = os.path.join(
        os.getcwd(), 
        'src/ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/rviz', 
        'mpc_vis.rviz'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        #启动mpc控制节点，指定包名、可执行文件名、节点名、参数配置
        Node(
            package='carla_shenlan_mpc_controller',
            executable='mpc_lateral_longitudinal',
            name='mpc_lateral_longitudinal',
            parameters=[lqr_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        #启动rviz2可视化节点
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
    ])
