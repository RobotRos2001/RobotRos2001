from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('TITA_DRL'), 
        'config', 
        'policy', 
        'params_57.yaml'
    )
    
    # 定义节点
    tita_drl_node = Node(
        package='TITA_DRL',          # 包名
        executable='tita_drl_obs57', # 可执行文件名
        name='tita_pointfoot_node',  # 节点名，需与 YAML 中一致
        output='screen',             # 输出到屏幕
        parameters=[config_file]     # 加载参数文件
    )
    
    # 返回 LaunchDescription，包含节点
    return LaunchDescription([
        tita_drl_node
    ])
