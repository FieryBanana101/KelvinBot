import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        self.subscription = self.create_subscription(Image, "/thermal_camera", self.process_image, 10)
        self.publisher = self.create_publisher(Image, "/final_image", 10)
        self.bridge = CvBridge()
        self.counter = 0

    def process_image(self, image):
        try:
            print("[Converting ROS image to cv2 image ... ]")
            test_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            print(f"[Received image {self.counter}]")
            self.counter += 1
        except CvBridgeError as e:
            print(e)
        

        try:
            print("[Converting cv2 image to ROS image ... ]\n")
            final_image = self.bridge.cv2_to_imgmsg(test_image, encoding='bgr8')
            print()
        except CvBridgeError as e:
            print(e)
        self.publisher.publish(final_image)



def main(args=None):
    print("Initializing....")
    rclpy.init(args=args)

    print("Creating image processor...")
    image_processor = ImageProcessor()

    print("Running image processor...")
    rclpy.spin(image_processor)

    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()