import os.path
import sys
import math


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node

def deg_to_rad(degrees):
    """将角度转换为弧度"""
    return degrees * math.pi / 180.0

def generate_launch_description():
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, MANUAL_BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE, LIVOX_MID360_CONFIG, LIVOX_MID360_CONFIG_NO_TILT, DEFAULT_NAMESPACE,
            SUPER_LIO_LIDAR_X, SUPER_LIO_LIDAR_Z,SUPER_LIO_LIDAR_TILT_ANGLE
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        MANUAL_BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        LIVOX_MID360_CONFIG_NO_TILT = ''
        DEFAULT_NAMESPACE = ''
        SUPER_LIO_LIDAR_X = -0.1
        SUPER_LIO_LIDAR_Z = -0.1
        SUPER_LIO_LIDAR_TILT_ANGLE = -30.0

    ld = LaunchDescription()


    declare_ns_arg = DeclareLaunchArgument(
        'ns',
        default_value=DEFAULT_NAMESPACE,
        description='Namespace for multi-robot support'
    )
    ld.add_action(declare_ns_arg)
    ns = LaunchConfiguration('ns')
    
    ns_map_frame = PythonExpression(["'map' if '", ns, "' == '' else str('", ns, "/map')"])
    ns_odom_frame = PythonExpression(["'odom' if '", ns, "' == '' else str('", ns, "/odom')"])
    ns_base_frame = PythonExpression(["'base_footprint' if '", ns, "' == '' else str('", ns, "/base_footprint')"])

    # ns_camera_init_frame = PythonExpression(["'camera_init' if '", ns, "' == '' else str('", ns, "/camera_init')"])
    ns_camera_init_frame = "camera_init"
    # ns_body_frame = PythonExpression(["'body' if '", ns, "' == '' else str('", ns, "/body')"])
    ns_body_frame = "body"
    
    ns_base_link_frame = PythonExpression(["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])
    
    




    package_path = get_package_share_directory('fast_lio_robosense')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='robosenseAiry.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Path to save the map PCD file'
    )

    map_file_path = LaunchConfiguration('map_file_path')

    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time,
                     'map_file_path': map_file_path}],
        prefix=['taskset -c 7'],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )











    # 添加静态变换发布器
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_map_frame, ns_odom_frame],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    static_transform_odom_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_camera_init',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_odom_frame, ns_camera_init_frame],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_camera_init)

    # camera_init -> body (里程计到机器人基坐标系的静态变换)
    static_transform_camera_init_to_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_camera_init_to_body',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_camera_init_frame, ns_body_frame],
        output='screen'
    )
    ld.add_action(static_transform_camera_init_to_body)

    body_to_rslidar_head_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_rslidar_head_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', '0.0', '0', ns_body_frame, 'rslidar_head'],
        output='screen'
    )
    ld.add_action(body_to_rslidar_head_tf)

    # rslidar_head -> base_link (机器人基坐标系到雷达坐标系的静态变换)
    rslidar_head_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_head_to_base_link_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-0.36615', '0', '0', '0', str(deg_to_rad(0)), '0', 'rslidar_head', ns_base_link_frame],
        output='screen'
    )
    ld.add_action(rslidar_head_to_base_link_tf)

    # rslidar_head -> rslidar_tail (雷达到雷达的静态变换)
    rslidar_head_to_rslidar_tail_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_head_to_rslidar_tail_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-0.7323', '0', '0.0', '0', str(deg_to_rad(0)), '0', 'rslidar_head', 'rslidar_tail'],
        output='screen'
    )
    ld.add_action(rslidar_head_to_rslidar_tail_tf)


    # # camera_init -> basefootprint (里程计到机器人基坐标系的静态变换)
    # static_transform_camera_init_to_base_footprint = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_camera_init_to_base_footprint',
    #     parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_camera_init_frame, ns_base_frame],
    #     output='screen'
    # )
    # ld.add_action(static_transform_camera_init_to_base_footprint)













    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_file_path_cmd)

    ld.add_action(fast_lio_node)
    # ld.add_action(rviz_node)

    return ld
