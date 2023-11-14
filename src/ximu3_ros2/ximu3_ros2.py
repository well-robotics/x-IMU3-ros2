import rclpy 
from rclpy.node import Node 
import ximu3 
import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header 
from rclpy.qos import QoSProfile

# create a ros2 node to publish the imu messages 
# Xiaobin Xiong 
# use the software to configure the message frequency 

def timestamp_format(timestamp):
    return "{:8.0f}".format(timestamp) + " us"


def int_format(value):
    return " " + "{:8.0f}".format(value)


def float_format(value):
    return " " + "{:8.3f}".format(value)


def string_format(string):
    return " \"" + string + "\""


class XIMU3_ROS2(Node):
    def __init__(self):
        super().__init__("ximu3_ros2")
        self.get_logger().info("start ximu-ros2 using usb connection")

        self.imu_publisher_ = self.create_publisher(Imu, 'ximu3/imu', 5)
        self.mag_publisher_ = self.create_publisher(MagneticField, 'ximu3/magnetic', 5)
        self.imu_msg = Imu() 
        # self.mag_msg = Magnetic()
        
        # self.connection = self.run(ximu3.UsbConnectionInfo("/dev/ttyACM0"))
        self.connection = ximu3.Connection(ximu3.UsbConnectionInfo("/dev/ttyACM0"))
        self.run()

    def run(self):
        # conn = ximu3.Connection(connection_info)
        print("here")
        self.connection.add_inertial_callback(self.inertial_callback)
        self.connection.add_magnetometer_callback(self.magnetometer_callback)
        self.connection.add_quaternion_callback(self.quaternion_callback)
        self.connection.add_high_g_accelerometer_callback(self.high_g_accelerometer_callback)

        print("Connecting to " + self.connection.get_info().to_string())
        if self.connection.open() != ximu3.RESULT_OK:
            raise Exception("Unable to open connection")
        
        print("XIMU 3Connection successful")
        self.connection.send_commands(['{"strobe":null}'], 2, 500)  # send command to strobe LED

    def inertial_callback(self, message):
        self.imu_msg.angular_velocity.x = message.gyroscope_x
        self.imu_msg.angular_velocity.y = message.gyroscope_y
        self.imu_msg.angular_velocity.z = message.gyroscope_z
        self.imu_msg.linear_acceleration.x = message.accelerometer_x
        self.imu_msg.linear_acceleration.y = message.accelerometer_y
        self.imu_msg.linear_acceleration.z = message.accelerometer_z
        self.imu_msg.header.stamp = self.get_clock().now().to_msg() 
        self.imu_publisher_.publish(self.imu_msg)
        # print(timestamp_format(message.timestamp) + " gyro" +
        #   float_format(message.gyroscope_x) + " deg/s" +
        #   float_format(message.gyroscope_y) + " deg/s" +
        #   float_format(message.gyroscope_z) + " deg/s" + " acc" +
        #   float_format(message.accelerometer_x) + " g" +
        #   float_format(message.accelerometer_y) + " g" +
        #   float_format(message.accelerometer_z) + " g")
       
    def magnetometer_callback(self,message):
        print(timestamp_format(message.timestamp) + "mag "+
            float_format(message.x) + " a.u." +
            float_format(message.y) + " a.u." +
            float_format(message.z) + " a.u.")
 

    def quaternion_callback(self,message):
        self.imu_msg.orientation.w = message.w
        self.imu_msg.orientation.x = message.x
        self.imu_msg.orientation.y = message.y
        self.imu_msg.orientation.z = message.z
         # print(timestamp_format(message.timestamp) + "quaternion" +
        #     float_format(message.w) +
        #     float_format(message.x) +
        #     float_format(message.y) +
        #     float_format(message.z))
 
    def high_g_accelerometer_callback(self, message):
        print(timestamp_format(message.timestamp) + " high g" +
            float_format(message.x) + " g" +
            float_format(message.y) + " g" +
            float_format(message.z) + " g")
 

 


def main(args=None):

    rclpy.init(args=args)
    node = XIMU3_ROS2()
    rclpy.spin(node)
    
    time.sleep(60)
    node.connection.close()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()