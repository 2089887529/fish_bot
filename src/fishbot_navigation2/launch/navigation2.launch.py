from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # == 定位包地址 ==
    fishbot_navigation2_dir = get_package_share_directory("fishbot_navigation2")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')   

    # == 声明参数,获取配置文件路径 ==
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境,其时间是通过/clock话题获取,而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'fishbot_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(fishbot_navigation2_dir, 'params', 'fishbot_nav.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # == 声明启动launch文件,传入:地图路径 是否使用仿真时间以及nav2参数文件 ==
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'param_file': nav2_param_path
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    return ld