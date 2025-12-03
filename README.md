# 该项目用于割草机器人整体软件架构 —— 更新 ing

## 项目结构
``` tree
/src
├── robot_control_pkg
├── rplidar_ros
└── wit_ros2_imu
```
- robot_control_pkg: 割草机器人控制软件包，负责 PC 端与下位机的通信，以及控制下位机，发送 left wheel and right wheel 的 velocity command，获取当前转速
- rplidar_ros: 割草机器人激光传感器软件包，负责激光传感器的初始化，以及获取激光传感器的数据 -> 使用**思岚 a2m6**激光雷达
- wit_ros2_imu: 割草机器人 IMU 软件包，负责 IMU 的初始化，以及获取 IMU 的数据 -> 使用**维特**十轴 IMU 模块

## 启动
启动方法：
1. 启动 IMU `ros2 run wit_ros2_imu wit_ros2_imu`
2. 启动激光传感器 `ros2 launch rplidar_ros rplidar_a2m8_launch.py`
3. 启动速度发布指令控制机器人移动底盘 `ros2 run robot_control_pkg robot_control_node` -> 接收 `/cmd_vel` topic
4. 启动里程计及里程计 TF 发布 `ros2 run robot_control_pkg pub_odom_tf_node` -> 启动后机器人往 `/odom` topic 发送 odometry msg 
5. 根据文档更新后续会更新 `launch` 文件以启动

## 端口映射方法
参考文章：[如果在 Ubuntu 系统中两个设备出现两个相同的端口号解决方案](https://blog.csdn.net/qq_57061492/article/details/137276611?ops_request_misc=%257B%2522request%255Fid%2522%253A%25224aa37dee76996781f45a8eb2d150f810%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=4aa37dee76996781f45a8eb2d150f810&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-137276611-null-null.142%5Ev102%5Econtrol&utm_term=ubuntu%20%E5%90%8C%E4%B8%80%E5%8E%82%E5%AE%B6%E7%AB%AF%E5%8F%A3%E5%8C%BA%E5%88%86&spm=1018.2226.3001.4187)
``` mower.rules
# set the udev rule , make the device_port be fixed by rplidar
#
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",ATTRS{devpath}=="5", MODE:="0777", SYMLINK+="imu"

KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",ATTRS{devpath}=="8", MODE:="0777", SYMLINK+="stm32"
```
e.g. -> **查询 KERNELS=="1-5" 下的 ATTRS{devpath}=="5"**，使用 `devpath` 去映射，尝试使用 `KERNERLS` 没有成功，未查明原因