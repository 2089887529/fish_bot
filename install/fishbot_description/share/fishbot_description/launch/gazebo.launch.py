from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
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
from launch.actions import ExecuteProcess
import os



def generate_launch_description():
    # 声明参数
    robot_name_in_model = 'fishbot'
    package_name = 'fishbot_description'
    urdf_name = 'fishbot_gazebo.urdf'

    ld = LaunchDescription()

    # 获取路径
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/urdf/{urdf_name}')

    # 启动Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 生成机器人
    spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path,'-z' '1.0' ], output='screen')

    # 发布机器人
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    return ld