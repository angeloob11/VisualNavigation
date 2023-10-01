import rclpy
import os
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf

def mean_iou(y_true, y_pred):
    
    num_classes = y_pred.shape[-1]
    y_true = tf.one_hot(tf.cast(y_true, tf.int32), num_classes, axis=-1)
    y_pred = tf.one_hot(tf.math.argmax(y_pred, axis=-1), num_classes, axis=-1)
    axes = (1, 2)
    intersection = tf.math.reduce_sum(y_true * y_pred, axis=axes)
    total = tf.math.reduce_sum(y_true, axis=axes) + tf.math.reduce_sum(y_pred, axis=axes)
    union = total - intersection
    is_class_present =  tf.cast(tf.math.not_equal(total, 0), dtype=tf.float32)
    num_classes_present = tf.math.reduce_sum(is_class_present, axis=1)
    iou = tf.math.divide_no_nan(intersection, union)
    iou = tf.math.reduce_sum(iou, axis=1) / num_classes_present
    mean_iou = tf.math.reduce_mean(iou)

    return mean_iou

model_dir = os.path.join(get_package_share_directory("nav_control"), 'segmentation_model')
model = tf.keras.models.load_model(model_dir, custom_objects={'mean_iou':mean_iou})

width = 224
height = 224

class Img_Node(Node):
    def __init__(self):
        super().__init__('img_node')

        self.img_sub = self.create_subscription(Image, "/camera_img", self.img_callback, qos_profile_sensor_data)
        self.img_pub = self.create_publisher(Image, "/seg_img", 100)
        self.cv_bridge = CvBridge()
    
    def img_callback(self, img):
        
        cv_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        cv_img = cv2.resize(cv_img, (width, height))
        input_tensor = tf.convert_to_tensor(cv_img, dtype=tf.float32)
        input_tensor = tf.expand_dims(input_tensor, 0)
        seg_img = model.predict(input_tensor)
        eval_img = self.cv_bridge.cv2_to_imgmsg(seg_img, "bgr8")
        eval_img.header = img.header
        self.img_pub.publish(eval_img)

def main(args=None):
    rclpy.init(args=args)
    Img_Treat_Node = Img_Node()
    rclpy.spin(Img_Treat_Node)
    Img_Treat_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




