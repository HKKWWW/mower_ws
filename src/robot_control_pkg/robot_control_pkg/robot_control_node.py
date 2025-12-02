# robot_control_pkg/robot_control_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

WHEEL_D = 0.346  # 轮距

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.sub = self.create_subscription(Twist,
                                             '/cmd_vel',
                                             self.cmd_vel_callback,
                                             10)

        # 初始化串口
        while True:
            try:
                self.ser = serial.Serial(port='/dev/tty_stm32',
                                        baudrate=9600,
                                        timeout=1.0)
                
                self.get_logger().info("--- Serial Port initialized, " \
                                        "ROS has connected STM32 ---")
                time.sleep(0.5)
                break

            except serial.serialutil.SerialException:
                self.get_logger().error("--- Unable to open serial port ---")
                self.ser = None
                time.sleep(1.0)

    @staticmethod
    def float_to_bytes(value: float) -> bytes:
        """
        将 float 转为 4 字节
        """
        return struct.pack('<f', value)  # float -> 4 byte

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel 回调
        """
        if self.ser is None:
            return

        self.get_logger().info("Recv V")

        v = msg.linear.x
        w = msg.angular.z

        # 运动学解算 (cm/s)
        vL = float((v - WHEEL_D * w) * 100.0)
        vR = float((v + WHEEL_D * w) * 100.0)

        # 限幅
        vL = min(max(vL, -15.0), 15.0)
        vR = min(max(vR, -15.0), 15.0)

        # float -> bytes
        bL = self.float_to_bytes(vL)
        bR = self.float_to_bytes(vR)

        # 构造数据帧
        send_left = bytes([0x56, 0x91, 0xaa]) + bL + bytes([0x11])   # 左轮
        send_right = bytes([0x56, 0x91, 0xbb]) + bR + bytes([0x11])  # 右轮

        # 写串口
        self.ser.write(send_left)
        time.sleep(0.001)
        self.ser.write(send_right)


def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()

    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass

    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
