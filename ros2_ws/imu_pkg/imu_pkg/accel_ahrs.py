import rclpy
from rclpy.node import Node
import serial, numpy as np, struct
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate, euler_from_quaternion


def send_rpy(r, p, y, pub, time, offset = [0, 0, 0]):
        
    send_quat(quaternion_from_euler(r, p, y), pub, time, offset)
    
def send_quat(q, pub, time, offset = [0, 0, 0]):

    msg = PoseStamped()
    msg.header.stamp = time.to_msg()
    msg.header.frame_id = 'map'
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    msg.pose.position.x = float(offset[0])
    msg.pose.position.y = float(offset[1])
    msg.pose.position.z = float(offset[2])
    
    pub.publish(msg)


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        
        self.ser = serial.Serial('/dev/ttyUSB0', 576000, timeout=1)
        self.dt = 10e-3
        
        self.gyro_state = np.array([0, 0, 0])
        
        self.mahony_Ki = 5e-5
        self.mahony_Kp = 5e-3
        self.mahony_state = np.array([0, 0, 0, 1])
        
        self.accel_pub = self.create_publisher(PoseStamped, 'accel_ahrs', 10)
        self.gyro_pub = self.create_publisher(PoseStamped, 'gyro_ahrs', 10)
        self.mahony_pub = self.create_publisher(PoseStamped, 'mahony_ahrs', 10)
        self.timer = self.create_timer(self.dt, self.callback)
        
    def callback(self, event = None):
        raw = self.ser.read(4 * 9)
        if len(raw) != 4 * 9:
            print ('Error: read {} bytes'.format(len(raw)))
            return
        ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
        print(f'Acceleration (m/s^2): ({ax: <8.3f}, {ay: <8.3f}, {az: <8.3f})'.rjust(65))
        print(f'Magnetic field (uT): ({mx: <8.3f}, {my: <8.3f}, {mz: <8.3f})'.rjust(65))
        print(f'Angular velocity (rad/s): ({gx: <8.3f}, {gy: <8.3f}, {gz: <8.3f})'.rjust(65))
        
        
        now = self.get_clock().now()
        
        # Accel
        send_rpy(np.arctan2(ay, az), np.arctan2(-ax, (ay*ay + az*az)**0.5), 0, self.accel_pub, now, offset=[-1, -1, 0])
        
        # Gyro
        self.gyro_state = self.gyro_state + np.array([gx, gy, gz]) * self.dt
        send_rpy(self.gyro_state[0], self.gyro_state[1], self.gyro_state[2], self.gyro_pub, now, offset=[-1, 1, 0])

        # Mahony
        sa = np.array([ax, ay, az])
        sm = np.array([mx, my, -mz])
        sw = np.array([gx, gy, gz])
        
        sa_ = np.array(quaternion_multiply(quaternion_multiply(quaternion_conjugate(self.mahony_state), np.array([0, 0, 9.81, 0])), self.mahony_state))
        sm_ = np.array(quaternion_multiply(quaternion_multiply(quaternion_conjugate(self.mahony_state), np.array([26.6068, 2.3557, 46.3964, 0])), self.mahony_state))
        
        sw_mest = np.cross(sa, sa_[:3]) + np.cross(sm, sm_[:3])
        sw_ = -self.mahony_Ki * sw_mest * self.dt
        
        sw_rqt = \
            np.array((gx, gy, gz, 0)) * self.dt * self.dt - \
            np.array((sw_[0], sw_[1], sw_[2], 0)) + \
            self.mahony_Kp * np.array((sw_mest[0], sw_mest[1], sw_mest[2], 0))
            
        self.mahony_state = self.mahony_state + 0.5 * np.array(quaternion_multiply(self.mahony_state, sw_rqt)) * self.dt
        # self.mahony_state /= (self.mahony_state[0]**2 + self.mahony_state[1]**2 + self.mahony_state[2]**2 + self.mahony_state[3]**2)**0.5
        
        send_quat(self.mahony_state, self.mahony_pub, now, offset=[1, -1, 0])
        
        



def main(args=None):
    rclpy.init(args=args)

    node = IMUPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()