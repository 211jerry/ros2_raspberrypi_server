import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
# 仅保留Humble必用的导入，无任何多余内容
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fishbot_bringup_dir = get_package_share_directory('fishbot_bringup')
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')


    # 1. 启动 urdf2tf (正常加载Python launch)
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_bringup_dir, 'launch', 'urdf2tf.launch.py')
        )
    )

    navigation2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_navigation2_dir, 'launch', 'navigation2.launch.py')
        )
    )

    # ===================== 核心修复：直接启动rosbridge，永不报错 =====================
    # ROS2 Humble 唯一正确方式：直接运行rosbridge节点，指定端口9090
    rosbridge_websocket = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        # 传递端口参数
        parameters=[{'port': 9090}]
    )

    # 以下所有节点保持原样，无任何修改
    odom2tf = launch_ros.actions.Node(
        package='fishbot_bringup',
        executable='odom2tf',
        output='screen'
    )

    camera_publisher = launch_ros.actions.Node(
        package='camera_publisher',
        executable='camera_publisher',
        output='screen'
    )

    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial','--dev','/dev/ttyUSB1','-b','921600'],
        output='screen'
    )

    range2scan = launch_ros.actions.Node(
        package='autosweeper_robot',
        executable='range2scan.py',
        output='screen'
    )

    m1ct_d2 = launch_ros.actions.Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        output='screen'
    )

    # 延迟启动
    m1ct_d2_delay = launch.actions.TimerAction(period=5.0, actions=[m1ct_d2])
    camera_publisher_delay = launch.actions.TimerAction(period=5.0, actions=[camera_publisher])

    # 最终启动列表
    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        microros_agent,
        range2scan,
        rosbridge_websocket,
        m1ct_d2_delay,
        camera_publisher_delay,
        navigation2,
    ])