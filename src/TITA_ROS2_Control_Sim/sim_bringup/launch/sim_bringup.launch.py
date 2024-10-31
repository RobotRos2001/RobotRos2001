#!/usr/bin/env python
import os
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

# 定义一个前缀，用于命名空间或控制器的标识
prefix = "tita"


def generate_launch_description():
    # 初始化声明的参数列表
    declared_arguments = []

    # 声明一个启动参数 "sim_env"，用于选择模拟环境
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_env",  # 参数名称
            default_value="gazebo",  # 默认值
            description="Select simulation environment",  # 参数说明
            choices=["webots", "gazebo"],  # 可选值
        )
    )

    # 包含 Webots 的控制器管理启动文件
    webots_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "webots_bridge"
                ),  # 获取 webots_bridge 包的共享目录
                "launch",
                "webots_bridge.launch.py",  # 启动文件路径
            )
        ),
        launch_arguments={
            "urdf": "robot.xacro",  # 传递 URDF 文件路径
            "yaml_path": "sim_bringup",  # 传递 YAML 配置文件路径
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("sim_env"), "' == 'webots'"]
            )  # 当选择的模拟环境为 Webots 时执行
        ),
    )

    # 包含 Gazebo 的控制器管理启动文件
    gazebo_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "gazebo_bridge"
                ),  # 获取 gazebo_bridge 包的共享目录
                "launch",
                "gazebo_bridge.launch.py",  # 启动文件路径
            )
        ),
        launch_arguments={
            "urdf": "robot.xacro",  # 传递 URDF 文件路径
            "yaml_path": "sim_bringup",  # 传递 YAML 配置文件路径
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("sim_env"), "' == 'gazebo'"]
            )  # 当选择的模拟环境为 Gazebo 时执行
        ),
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",  # 控制器管理包
        executable="spawner",  # 可执行文件
        arguments=[
            "joint_state_broadcaster",  # 要启动的控制器名称
            "--controller-manager",  # 指定控制器管理器
            prefix + "/controller_manager",  # 控制器管理器的命名空间
        ],
    )

    # 启动 IMU 传感器广播器
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",  # 控制器管理包
        executable="spawner",  # 可执行文件
        arguments=[
            "imu_sensor_broadcaster",  # 要启动的控制器名称
            "--controller-manager",  # 指定控制器管理器
            prefix + "/controller_manager",  # 控制器管理器的命名空间
        ],
    )

    # 启动施力控制器
    effort_controller = Node(
        package="controller_manager",  # 控制器管理包
        executable="spawner",  # 可执行文件
        arguments=[
            "effort_controller",  # 要启动的控制器名称
            "--controller-manager",  # 指定控制器管理器
            prefix + "/controller_manager",  # 控制器管理器的命名空间
        ],
    )

    # 启动模板控制器（当前已注释掉）
    template_controller = Node(
        package="controller_manager",  # 控制器管理包
        # output='screen',  # 取消注释以在屏幕上输出日志
        executable="spawner",  # 可执行文件
        arguments=[
            "template_ros2_controller",  # 要启动的控制器名称
            "--controller-manager",  # 指定控制器管理器
            prefix + "/controller_manager",  # 控制器管理器的命名空间
        ],
    )

    # 启动模板控制器（当前已注释掉）
    position_controller = Node(
        package="controller_manager",  # 控制器管理包
        # output='screen',  # 取消注释以在屏幕上输出日志
        executable="spawner",  # 可执行文件
        arguments=[
            "position_controller",  # 要启动的控制器名称
            "--controller-manager",  # 指定控制器管理器
            prefix + "/controller_manager",  # 控制器管理器的命名空间
        ],
    )

    # 返回启动描述，包括声明的参数和所有启动的节点
    return LaunchDescription(
        declared_arguments
        + [
            webots_controller_manager_launch,  # Webots 启动文件
            gazebo_controller_manager_launch,  # Gazebo 启动文件
            joint_state_broadcaster_spawner,  # 关节状态广播器
            imu_sensor_broadcaster_spawner,  # IMU 传感器广播器
            # effort_controller,  # 施力控制器
            # template_controller,  # 模板控制器
            position_controller,
        ]
    )
