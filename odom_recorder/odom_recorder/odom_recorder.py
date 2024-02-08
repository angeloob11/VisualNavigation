import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
import csv

row_header = ['x', 'y', 'z']

class odom_recorder(Node):
    def __init__(self):
        super().__init__('odom_recorder_node')

        self.odom_sub = self.create_subscription(Odometry, '/model/marble_husky_sensor_config_1/odometry', self.odom_callback, qos_profile_system_default)
        with open('Odom_writer.csv', 'w') as file:
            file_writer = csv.writer(file)
            file_writer.writerow(row_header)
        

    def odom_callback(self, msg):
        POSITION = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        with open('Odom_writer.csv', 'a+') as file:
            file_writer = csv.writer(file)
            file_writer.writerow(POSITION)
            print('COORDENADAS ESCRITAS')


def main(args=None):
    rclpy.init(args=args)
    Odom_Node = odom_recorder()
    rclpy.spin(Odom_Node)
    Odom_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()