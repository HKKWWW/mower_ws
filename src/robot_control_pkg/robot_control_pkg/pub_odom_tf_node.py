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

WHEEL_D = 0.346  # 轮距 (m)

class OdomSerialImuNode(Node):
    def __init__(self):
        super().__init__('pub_odom_tf_node')

        self.create_subscription(Imu, '/imu', self.imu_callback, 3)  # IMU 订阅
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)  # odom 发布
        self.tf_broadcaster = TransformBroadcaster(self)  # tf 广播

        # 初始化串口
        while True:
            try:
                self.ser = serial.Serial(port='/dev/stm32',
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
        # self.get_logger().info("yaw: %.2f" % self.yaw)

    def read_wheel(self):
        if self.ser is None:
            return False

        # 向 MCU 发送请求
        send = bytes([0x56, 0x91, 0xcc, 0, 0, 0, 0, 0x11])
        self.ser.write(send)
        time.sleep(0.005)

        if self.ser.in_waiting >= 8:
            data = self.ser.read(16)
            if len(data) < 16:
                return False
            # self.get_logger().info("received: %s" % data)
            if data[0] == 0x56 and data[1] == 0x91 and data[7] == 0x11 and \
                 data[8] == 0x56 and data[9] == 0x91 and data[15] == 0x11:
                valueL = struct.unpack('<f', data[2:6])[0]
                valueR = struct.unpack('<f', data[10:14])[0]
                self.vL = valueL
                self.vR = valueR
                # self.get_logger().info("vL: %.2f, vR: %.2f" % (valueL, valueR))
                return True
        else:
            return False

    def timer_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        if not self.read_wheel():
            self.get_logger().warning("当前帧无法读取轮速数据")
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
