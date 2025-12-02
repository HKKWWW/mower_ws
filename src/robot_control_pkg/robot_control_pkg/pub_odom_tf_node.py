# robot_control_pkg/pub_odom_tf_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import time

WHEEL_D = 0.346  # 轮距 (m); 根据你 MCU 宏 WHEEL_D

class OdomSerialImuNode(Node):
    def __init__(self):
        super().__init__('odom_serial_imu_node')

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)  # IMU 订阅
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)  # odom 发布
        self.tf_broadcaster = TransformBroadcaster(self)  # tf 广播

        # 初始化串口
        while True:
            try:
                self.ser = serial.Serial(port='/dev/tty_stm32',
                                        baudrate=9600,
                                        timeout=1.0)
                
                self.get_logger().info("--- Start publishing odometry msg and TF ---")
                time.sleep(0.5)
                break

            except serial.serialutil.SerialException:
                self.get_logger().error("--- Unable to open serial port ---")
                self.ser = None
                time.sleep(1.0)

        # 状态
        self.yaw = 0.0  # IMU
        self.last_time = self.get_clock().now()

        self.vL = None
        self.vR = None

        self.x = 0.0
        self.y = 0.0

        # 循环定时 (20 Hz)
        self.timer = self.create_timer(1.0/20.0, self.timer_loop)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        # 二维平面
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def read_wheel(self):
        if self.ser is None:
            return False

        # 向 MCU 发送请求
        send = bytes([0x56, 0x91, 0xcc, 0, 0, 0, 0, 0x11])
        self.ser.write(send)
        time.sleep(0.001)

        # 等待数据
        # 假设 MCU 依次返回左右轮速度帧
        got_L = False
        got_R = False
        vL = None
        vR = None

        t0 = time.time()
        timeout = 0.05  # 50 ms 超时
        while time.time() - t0 < timeout:
            if self.ser.in_waiting >= 8:
                data = self.ser.read(8)
                if len(data) != 8:
                    continue
                if data[0] == 0x56 and data[1] == 0x91 and data[7] == 0x11:
                    kind = data[2]
                    value = struct.unpack('<f', data[3:7])[0]
                    if kind == 0x55:
                        got_L = True
                        vL = value
                    elif kind == 0x66:
                        got_R = True
                        vR = value
                if got_L and got_R:
                    break

        if got_L and got_R:
            self.vL = vL
            self.vR = vR
            return True
        else:
            return False

    def timer_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        if not self.read_wheel():
            # 没拿到完整轮速，就跳过这次 odom 更新
            return

        # 轮速是 cm/s → 转为 m/s
        vL_ms = self.vL / 100.0
        vR_ms = self.vR / 100.0

        # 线速度和角速度 (从轮子)
        vx = (vL_ms + vR_ms) / 2.0
        vth_wheel = (vR_ms - vL_ms) / (2.0 * WHEEL_D)

        # 用 IMU yaw 作为当前 heading
        th = self.yaw

        # 根据 vx 和 yaw 积分更新 x, y
        dx = vx * math.cos(th) * dt
        dy = vx * math.sin(th) * dt
        self.x += dx
        self.y += dy

        # 发布 tf
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # yaw → quaternion
        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 发布 odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth_wheel

        self.odom_pub.publish(odom)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdomSerialImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
