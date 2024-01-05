import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy import signal

class Img_Node_CV(Node):
    def __init__(self):
        super().__init__('img_node')

        self.img_sub = self.create_subscription(Image, "/camera_img", self.img_callback, qos_profile_sensor_data)
        self.theta_pub = self.create_publisher(Float64, "/theta", 100)
        self.finish_pub = self.create_publisher(Bool, "/finish", 100)
        self.cv_bridge = CvBridge()

    def img_treatment(self, img):
        edges = cv2.Canny(img, 100, 200, None, 3, cv2.DIST_L2)
        normalized = cv2.normalize(edges, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        column_intensity = normalized.sum(axis=0)
        window_size = 30
        window = np.ones((window_size,)) / window_size
        smoothed = np.convolve(column_intensity, window, mode="valid")
        indices = signal.argrelmin(smoothed)[0]
        mins = smoothed[indices]
        min_indx = np.where(mins == min(mins))[0]
        return indices[min_indx], np.mean(mins)
    
    def img_callback(self, img_msg):
        #READ THE IMAGE WITH OPENCV BRIDGE
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        #APPLY CANNY AND OTHER OPERATIONS ANG GET THE VECTOR DATA
        x, status = self.img_treatment(img)
        y = 450
        dy = 600 - y
        dx = x - 400
        theta = np.arctan2(dx,dy)
        #PUBLIS THE DATA
        theta_msg = Float64()
        theta_msg.data = float(theta[0])
        finish_msg = Bool()
        if(status < 20):
            finish_msg.data = True
        else:
            finish_msg.data = False

        self.theta_pub.publish(theta_msg)
        self.finish_pub.publish(finish_msg)

def main(args=None):
    rclpy.init(args=args)
    Img_Treat_Node = Img_Node_CV()
    rclpy.spin(Img_Treat_Node)
    Img_Treat_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




