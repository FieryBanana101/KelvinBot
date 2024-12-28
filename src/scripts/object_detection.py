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
            current_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            print(f"[Received image {self.counter}]")
            self.counter += 1
        except CvBridgeError as e:
            print(e)
        
        coloured_thermal_image = cv2.normalize(current_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        coloured_thermal_image = cv2.applyColorMap(coloured_thermal_image, cv2.COLORMAP_JET)

        binary_mask = cv2.inRange(coloured_thermal_image, (0,0,100), (50,170,256))

        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            red = coloured_thermal_image[y][x][2]
            green = coloured_thermal_image[y][x][1]
            blue = coloured_thermal_image[y][x][0]
            temperature =  (0.299 * red + 0.587 * green + 0.114 * blue) / 255.0
            temperature = 250 + temperature * 750

            cv2.rectangle(coloured_thermal_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(coloured_thermal_image, f"{temperature:.2f} Celcius", (x, y - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
    
        try:
            print("[Converting cv2 image to ROS image ... ]\n")
            final_image = self.bridge.cv2_to_imgmsg(coloured_thermal_image, encoding='bgr8')
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