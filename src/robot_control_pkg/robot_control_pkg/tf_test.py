import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DynamicTfPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        
        # 初始化TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 绕圈参数
        self.radius = 1.0  # 绕圈半径（米）
        self.angular_speed = 0.5  # 角速度（rad/s），越小绕圈越慢
        self.start_time = self.get_clock().now().nanoseconds * 1e-9  # 起始时间
        
        # 定时器：10Hz发布TF（动态更新）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("动态TF发布节点已启动！base_link将绕odom帧原点绕圈")

    def timer_callback(self):
        # 计算当前时间差
        now = self.get_clock().now()
        current_time = now.nanoseconds * 1e-9
        dt = current_time - self.start_time
        
        # 计算绕圈的位置和姿态（圆形轨迹）
        # x = r*sin(ωt)，y = r*cos(ωt) → 顺时针绕圈
        x = self.radius * math.sin(self.angular_speed * dt)
        y = self.radius * math.cos(self.angular_speed * dt)
        # 姿态角（yaw）随时间变化，和绕圈角速度一致
        yaw = self.angular_speed * dt  # 弧度
        
        # 构造TF消息
        t = TransformStamped()
        # 设置时间戳（必须用节点当前时间，否则TF会失效）
        t.header.stamp = now.to_msg()
        # 父帧：odom，子帧：base_link（符合ROS导航规范）
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 平移（绕圈的x/y，z=0）
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # 旋转：仅yaw角，转为四元数
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # 发布TF
        self.tf_broadcaster.sendTransform(t)
        
        # 打印日志（可选，看实时位置）
        self.get_logger().debug(f"发布TF: x={x:.2f}m, y={y:.2f}m, yaw={yaw:.2f}rad")

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建节点
    node = DynamicTfPublisher()
    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获Ctrl+C，优雅退出
        node.get_logger().info("节点被手动停止")
    finally:
        # 销毁节点
        node.destroy_node()
        # 关闭ROS2
        rclpy.shutdown()

if __name__ == '__main__':
    main()