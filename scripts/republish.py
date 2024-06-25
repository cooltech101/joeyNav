import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualnav_transformer.deployment.src.topic_names import (
    IMAGE_TOPIC,
    SAMPLED_ACTIONS_TOPIC,
    WAYPOINT_TOPIC,
)
class ImageWaypointPlotterNode(Node):
    def __init__(self):
        super().__init__('image_waypoint_plotter')
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10)
        self.waypoint_subscription = self.create_subscription(
            Float32MultiArray,
            WAYPOINT_TOPIC,
            self.waypoint_callback,
            10)
        
        # Publisher
        self.publisher = self.create_publisher(Image, '/image_with_waypoint', 10)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_waypoint = None

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_and_publish()

    def waypoint_callback(self, msg):
        if len(msg.data) >= 2:
            # rescale image coordinates to match the image size

            self.latest_waypoint = (int(msg.data[0]), int(msg.data[1]))
            if self.latest_image is not None:
                self.latest_waypoint = (int(msg.data[0] * self.latest_image.shape[0]), int(msg.data[1] * self.latest_image.shape[1]))

        self.process_and_publish()

    def process_and_publish(self):
        if self.latest_image is not None and self.latest_waypoint is not None:
            image_with_waypoint = self.latest_image.copy()
            
            # Draw the waypoint on the image
            cv2.circle(image_with_waypoint, self.latest_waypoint, 5, (0, 255, 0), -1)
            
            # Convert back to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(image_with_waypoint, encoding="bgr8")
            self.publisher.publish(ros_image)
            self.get_logger().info(f'Published image with waypoint at {self.latest_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageWaypointPlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
