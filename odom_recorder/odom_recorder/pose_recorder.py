import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_msgs.msg import TFMessage
import csv

row_header = ['x', 'y', 'z']

class pose_recorder(Node):
    def __init__(self):
        super().__init__('pose_recorder_node')

        self.odom_sub = self.create_subscription(TFMessage, '/world/field/dynamic_pose/info', self.pose_callback, qos_profile_system_default)
        with open('Pose_writer.csv', 'w') as file:
            file_writer = csv.writer(file)
            file_writer.writerow(row_header)
    
    def pose_callback(self, msg):
        TF_msg = msg.transforms[0]
        POSITION = [TF_msg.transform.translation.x, TF_msg.transform.translation.y, TF_msg.transform.translation.z]
        with open('Pose_writer.csv', 'a+') as file:
            file_writer = csv.writer(file)
            file_writer.writerow(POSITION)
            print('POSE ESCRITA')


def main(args=None):
    rclpy.init(args=args)
    Pose_Node = pose_recorder()
    rclpy.spin(Pose_Node)
    Pose_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()