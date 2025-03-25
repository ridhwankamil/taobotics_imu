import rclpy
from rclpy.node import Node
import serial
import struct
import math
import serial.tools.list_ports
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('gra_normalization', True)
        self.declare_parameter('frame_id', 'imu')
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.gra_normalization = self.get_parameter('gra_normalization').value
        self.frame_id = self.get_parameter('frame_id').value

        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'mag', 10)
        
        self.timer = self.create_timer(1.0/300, self.read_serial_data)

        self.buff = {}
        self.key = 0
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True]
        self.data_right_count = 0

        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.5)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            exit(1)

    def read_serial_data(self):
        try:
            buff_count = self.serial_port.in_waiting
            if buff_count > 0:
                buff_data = self.serial_port.read(buff_count)
                for byte in buff_data:
                    self.handle_serial_data(byte)
        except Exception as e:
            self.get_logger().error(f"IMU connection lost: {e}")
            exit(1)

    def check_sum(self, list_data, check_data):
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data_str = ''.join([format(raw_data[i + j], '02x') for j in range(4)])
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data_str))[0])
        ieee_data.reverse()
        return ieee_data

    def handle_serial_data(self, raw_data):
        """ Handle raw data byte by byte"""
        if self.key == 0 and raw_data != 0xaa: #check if the first byte is 0xaa
            return
        
        self.buff[self.key] = raw_data
        self.key += 1
        
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return
        
        data_buff = list(self.buff.values())
        
        if self.buff[2] == 0x2c and self.pub_flag[0]:
            if self.check_sum(data_buff[2:47], data_buff[47:49]):
                data = self.hex_to_ieee(data_buff[7:47])
                self.angularVelocity = data[1:4]
                self.acceleration = data[4:7]
                self.magnetometer = data[7:10]
            self.pub_flag[0] = False
        elif self.buff[2] == 0x14 and self.pub_flag[1]:
            if self.check_sum(data_buff[2:23], data_buff[23:25]):
                data = self.hex_to_ieee(data_buff[7:23])
                self.angle_degree = data[1:4]
            self.pub_flag[1] = False
        
        self.buff = {}
        self.key = 0
        self.pub_flag[0] = self.pub_flag[1] = True
        
        stamp = self.get_clock().now().to_msg()
        
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        
        angle_radian = [angle * math.pi / 180 for angle in self.angle_degree]
        qua = self.quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])
        
        imu_msg.orientation.x = qua._x
        imu_msg.orientation.y = qua._y
        imu_msg.orientation.z = qua._z
        imu_msg.orientation.w = qua._w
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = self.angularVelocity
        
        acc_k = math.sqrt(sum(a ** 2 for a in self.acceleration)) or 1
        scale_factor = -9.8 / acc_k if self.gra_normalization else -9.8
        imu_msg.linear_acceleration.x = self.acceleration[0] * scale_factor
        imu_msg.linear_acceleration.y = self.acceleration[1] * scale_factor
        imu_msg.linear_acceleration.z = self.acceleration[2] * scale_factor
        
        mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = self.magnetometer
        
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

    def quaternion_from_euler(self,roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q._w = cy * cp * cr + sy * sp * sr
        q._x = cy * cp * sr - sy * sp * cr
        q._y = sy * cp * sr + cy * sp * cr
        q._z = sy * cp * cr - cy * sp * sr
        return q 


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
