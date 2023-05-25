import rclpy
from rclpy.node import Node
import serial, numpy as np, struct
from geometry_msgs.msg import PoseStamped


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        self.ser = serial.Serial('/dev/ttyACM0', 576000, timeout=1)
        self.dt = 10e-3
        
        self.pub = self.create_publisher(PoseStamped, 'imu', 10)
        self.timer = self.create_timer(self.dt, self.callback)
        
    def callback(self, event=None):
        raw = self.ser.read(4 * 4)
        if len(raw) != 4 * 4:
            self.get_logger().error(f'Error: read {len(raw)} bytes')
            return
        q = struct.unpack('f' * 4, raw)
        
        now = self.get_clock().now()
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        
        self.pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)

    node = IMUPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()