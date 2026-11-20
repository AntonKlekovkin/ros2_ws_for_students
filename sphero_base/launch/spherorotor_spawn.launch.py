import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
import xacro
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_name='sphero_base'

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    #urdf_file = os.path.join(pkg_path,'urdf','spherorobot_rotor_v1.urdf.xacro')
    #urdf_path = PathJoinSubstitution( [FindPackageShare(package_name), "urdf" , "spherorobot_rotor_v1.urdf"])
    #robot_description_config = xacro.process_file(urdf_file)
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("sphero_base"), "urdf", "spherorobot_rotor_v1.urdf.xacro"]
    )
    

    # Путь к библиотекам плагина
    pkg_prefix = get_package_prefix(package_name)
    plugin_path = os.path.join(pkg_prefix, 'lib')
    
    # Установить переменную окружения для Gazebo
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path
    )

    simTimeArg = DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true')

    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
                    'publish_frequency': 50.0,  # Частота публикации (Гц)
                    'ignore_timestamp': False,  # Игнорировать временные метки
                    'frame_prefix': ''  # Префикс для фреймов
                }
            ]
        )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states_source'],  # откуда брать данные
            'rate': 50  # частота публикации
        }]
    )
    
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            #'-world', current_world,
            '-name', 'sphero',
            '-topic', 'robot_description',
            'allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.12', '-R', '0.0' #'0.1736'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_spawn = TimerAction(
        period=1.0,
        actions=[spawn]
    )
    
    
    return LaunchDescription([        
        simTimeArg,
        #gz_plugin_path,
        node_robot_state_publisher,
        node_joint_state_publisher,
        delayed_spawn
    ])