import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        # Assina a imagem da câmera
        self.image_sub = self.create_subscription(
            Image,
            '/drone/camera/image_raw',  # ou outro nome, dependendo da sua simulação
            self.image_callback,
            10
        )

        # Publica velocidade no MAVROS
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.lost_line_counter = 0
        self.max_lost_frames = 10

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        M = cv2.moments(mask)
        twist = Twist()

        if M["m00"] > 1000:
            self.lost_line_counter = 0
            cx = int(M["m10"] / M["m00"])
            error = cx - (cv_image.shape[1] // 2)

            # Controle simples
            twist.linear.x = 0.3
            twist.angular.z = -float(error) / 100.0
            self.cmd_pub.publish(twist)
        else:
            self.lost_line_counter += 1
            if self.lost_line_counter > self.max_lost_frames:
                self.stop_drone()

        # Visualização (opcional)
        cv2.imshow("Máscara Azul", mask)
        cv2.waitKey(1)

    def stop_drone(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("Linha azul não detectada. Drone parado.")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
