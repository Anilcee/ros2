import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscriber = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.window_name = "Camera"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info('Image subsription started')

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1)
            if key == 27:
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
