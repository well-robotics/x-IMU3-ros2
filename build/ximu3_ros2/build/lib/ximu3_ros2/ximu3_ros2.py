import rclpy 
from rclpy.node import Node 
import ximu3 

 
# create a ros2 node to publish the imu messages 

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

        self.connection = self.run(ximu3.UsbConnectionInfo("/dev/ttyACM0"))

    def run(self, connection_info):
        conn = ximu3.Connection(connection_info)

        conn.add_inertial_callback(self.inertial_callback)

    def inertial_callback(self, message):
        print(timestamp_format(message.timestamp) + " gyro" +
          float_format(message.gyroscope_x) + " deg/s" +
          float_format(message.gyroscope_y) + " deg/s" +
          float_format(message.gyroscope_z) + " deg/s" + " acc" +
          float_format(message.accelerometer_x) + " g" +
          float_format(message.accelerometer_y) + " g" +
          float_format(message.accelerometer_z) + " g")
        


def main(args=None):

    rclpy.init(args=args)
    node = XIMU3_ROS2()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()