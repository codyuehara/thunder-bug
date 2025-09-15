import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
import torch 
import cv2
import numpy as np
from cv_bridge import CvBridge

class GateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # load CNN (TorchScript for speed)
        #self.model = torch.jit.load("gate_detector.pt")
        #self.model.eval()

    def image_callback(self, msg):
        # TODO
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_coding='bgr8')
        self.get_logger().info(f"Received image of shape {cv_image.shape}")
        cv2.imshow("Input Image", cv_image)
        cv.waitKey(1)
        return None    
    
    def decode_corners(self, output, original_shape):
        """
        output: [8, H, W] tensor with [corner heatmaps + PAFs]
        """
        
        # TODO
        heatmaps = output[:4]
        pafs = output[4:]
        corners = []
        

def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
