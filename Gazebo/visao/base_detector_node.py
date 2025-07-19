import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # For receiving camera image
from geometry_msgs.msg import Point # For publishing base type and position
from cv_bridge import CvBridge # For converting ROS Image messages to OpenCV images

import cv2
import numpy as np

# --- Definition of Base Types (for mapping with takeoff_node.py) ---
# These values should match the ones expected in takeoff_node.py
# Using integers for mapping to Point.x is typical.
class BaseType:
    SQUARE_BROWN = 1
    TRIANGLE_BLUE = 2
    HEXAGON_RED = 3
    # Add other base types as needed

class BaseDetector(Node):
    def __init__(self):
        super().__init__('base_detector_node')

        # Create a subscriber for the camera image topic
        # The topic may vary, check what your drone publishes (e.g., /camera/image_raw, /depth_camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # <--- VERIFY AND ADJUST THIS TOPIC!
            self.image_callback,
            10 # QoS depth
        )
        self.subscription # Prevents unused variable warning

        # Create a publisher for the detected base data (type and position in image)
        # Point.x will be the base type (an integer mapped from BaseType)
        # Point.y and Point.z will be the (x, y) coordinates of the base's center in the image
        self.publisher = self.create_publisher(Point, '/drone/base_data', 10)

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Flag to ensure we publish the base only once per detection
        # or until the drone moves away from the current base.
        # For this mission, we assume we only need the first correct base detected.
        self.base_detected_and_published = False

        self.get_logger().info('Base Detector Node has been started.')

    def image_callback(self, msg):
        """
        Callback that is called every time a new image is received.
        Processes the image to detect base shapes and colors.
        """
        if self.base_detected_and_published:
            # If a base has already been detected and published, do not process further until reset.
            return

        try:
            # Convert ROS Image message to an OpenCV image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get image dimensions (though not directly used for publishing, can be useful for debugging/scaling)
        height, width, _ = cv_image.shape

        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # --- Define HSV Color Ranges for Bases ---
        # These values are examples and NEED to be calibrated for your environment and colors.
        # Use color calibration tools or trial and error.

        # Example Brown color (for SQUARE_BROWN)
        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([20, 255, 200])

        # Example Blue color (for TRIANGLE_BLUE)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Example Red color (for HEXAGON_RED)
        # Note: Red can have two ranges in HSV (near 0 and near 180)
        lower_red1 = np.array([0, 100, 20])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 20])
        upper_red2 = np.array([180, 255, 255])

        # --- Shape and Color Detection Logic ---
        detected_base_info = None # (base_type_enum, center_x, center_y)

        # Process for Brown (Square)
        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
        contours_brown, _ = cv2.findContours(mask_brown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_brown:
            area = cv2.contourArea(contour)
            if area > 500: # Adjust minimum area size to filter noise
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                
                # For a square, we expect 4 vertices
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w)/h
                    # Check if it's approximately a square (aspect ratio close to 1)
                    if 0.8 <= aspect_ratio <= 1.2:
                        center_x = x + w / 2
                        center_y = y + h / 2
                        detected_base_info = (BaseType.SQUARE_BROWN, center_x, center_y)
                        break # Exit the 'for contour in contours_brown' loop

        # Process for Blue (Triangle) - only if no brown base was found yet
        if not detected_base_info:
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_blue:
                area = cv2.contourArea(contour)
                if area > 500: # Adjust minimum area size
                    perimeter = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                    
                    # For a triangle, we expect 3 vertices
                    if len(approx) == 3:
                        x, y, w, h = cv2.boundingRect(approx)
                        center_x = x + w / 2
                        center_y = y + h / 2
                        detected_base_info = (BaseType.TRIANGLE_BLUE, center_x, center_y)
                        break # Exit the 'for contour in contours_blue' loop

        # Process for Red (Hexagon) - only if no previous base was found yet
        if not detected_base_info:
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.add(mask_red1, mask_red2) # Combine the two red masks
            
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_red:
                area = cv2.contourArea(contour)
                if area > 500: # Adjust minimum area size
                    perimeter = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                    
                    # For a hexagon, we expect 6 vertices
                    if len(approx) == 6:
                        x, y, w, h = cv2.boundingRect(approx)
                        center_x = x + w / 2
                        center_y = y + h / 2
                        detected_base_info = (BaseType.HEXAGON_RED, center_x, center_y)
                        break # Exit the 'for contour in contours_red' loop

        # --- Publish Detected Base Data ---
        if detected_base_info:
            base_type_enum, center_x, center_y = detected_base_info
            
            point_msg = Point()
            point_msg.x = float(base_type_enum) # Base type (as an integer)
            point_msg.y = float(center_x)       # X position of base center in image
            point_msg.z = float(center_y)       # Y position of base center in image
            
            self.publisher.publish(point_msg)
            self.get_logger().info(f"Published Detected Base: Type {base_type_enum}, Center X: {center_x:.2f}, Y: {center_y:.2f}")
            
            # Mark that a base has been detected and published
            self.base_detected_and_published = True

        # Optional: Show the processed image (for debugging)
        # cv2.imshow("Base Detector Feed", cv_image)
        # cv2.waitKey(1) # Required to update OpenCV windows

def main(args=None):
    rclpy.init(args=args)
    base_detector = BaseDetector()
    rclpy.spin(base_detector) # Keep the node running and processing callbacks
    base_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()