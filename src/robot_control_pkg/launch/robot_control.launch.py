# robot_control_pkg/robot_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 创建启动描述
    ld = LaunchDescription()

    # 启动 pub_odom_tf_node 节点
    pub_odom_tf_node = Node(
        package='robot_control_pkg',
        executable='pub_odom_tf_node',
        name='pub_odom_tf_node',
        output='screen',
        emulate_tty=True,
    )

    # 启动 robot_control_node 节点
    robot_control_node = Node(
        package='robot_control_pkg',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        emulate_tty=True,
    )

    # 启动 mower_description.launch.py 
    mower_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_description'),
                'launch',
                'mower_description.launch.py'
            )
        )
    )

    # 启动 wit_ros2_imu 节点
    wit_ros2_imu = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='wit_ros2_imu',
        output='screen',
        emulate_tty=True,
    )

    # 添加节点到启动描述中
    ld.add_action(pub_odom_tf_node)
    ld.add_action(robot_control_node)
    ld.add_action(wit_ros2_imu)
    ld.add_action(mower_description_launch)

    return ld
