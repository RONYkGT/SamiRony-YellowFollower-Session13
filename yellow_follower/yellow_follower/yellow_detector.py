import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowDetector(Node):
    def __init__(self):
        super().__init__('yellow_detector')
        self.img_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your camera image topic
            self.image_callback,
            10)
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your camera image topic
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            '/yellow_detected/image',  # Topic to publish the processed image
            10)
        
        self.timer = self.create_timer(0.1, self.control_robot)
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.object_x = 0.0
        self.sphere_found = False
        self.msg = None


    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected yellow areas
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Minimum area to filter noise
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw green box
                self.object_x = x + w / 2
            self.sphere_found = True
        else:
            self.sphere_found = False

        # Convert the OpenCV image back to ROS Image message
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the processed image
        self.publisher.publish(processed_image_msg)

    def scan_callback(self, msg):
        self.msg = msg
        
    def control_robot(self):
        self.get_logger().info("target x= %f" % self.object_x)
        cmd = Twist()

        if self.msg is None or not self.msg.ranges:
         return

        min_distance = min(self.msg.ranges)

        if min_distance < 0.5 and self.sphere_found:
            # Stop if too close to any object
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.sphere_found:
            self.get_logger().info("found")
            if self.object_x is not None:
                # Calculate the center of the image
                center_x = 1920 / 2
                
                # Rotate to align with the target
                if self.object_x < center_x - 700:  # Allowing more tolerance
                    cmd.angular.z = 0.5
                elif self.object_x > center_x + 700:
                    cmd.angular.z = -0.5
                else:
                    cmd.angular.z = 0.0

                # Move forward when aligned
                cmd.linear.x = 0.5
            else:
                cmd.angular.z = 0.5
                cmd.linear.x = 0.0
        else:
            # Rotate in place if no sphere found
            cmd.angular.z = 0.5
            cmd.linear.x = 0.0

        # Publish the velocity command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    yellow_color_detector = YellowDetector()
    rclpy.spin(yellow_color_detector)
    yellow_color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
