#一键启动
source install/setup.bash
ros2 launch fishbot_bringup bringup.launch.py

    #启动激光雷达
    source install/setup.bash
    ros2 run m1ct_d2 m1ct_d2 

    #启动摄像机
    source install/setup.bash
    ros2 run camera_publisher camera_publisher

    #启动端口
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

    #启动tf变换
    source install/setup.bash
    ros2 run fishbot_bringup odom2tf

    #启动描述文件
    source install/setup.bash
    ros2 launch fishbot_bringup urdf2tf.launch.py

#启动slam建图
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/fishbot_bringup/config/slam_toolbox.yaml use_sim_time:=False

#启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

#保存地图
ros2 run nav2_map_server map_saver_cli -t map -f room

#启动导航
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=False

# 停止吸尘器/扫地刷
ros2 topic pub --once /clean std_msgs/msg/Bool "{data: false}"


# 开始全覆盖清扫
source install/setup.bash
ros2 run autosweeper_robot sweeper_node

# 开始快速清扫（条带清扫）
source install/setup.bash
ros2 run autosweeper_robot linesweeper

# 开始沿边清扫
source install/setup.bash
ros2 run autosweeper_robot alongedge

# 开始弓形清扫
source install/setup.bash
ros2 run autosweeper_robot archsweeper

# 启动安卓服务器
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090