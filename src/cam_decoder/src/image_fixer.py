import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageFixer(Node):
    def __init__(self):
        super().__init__('image_fixer_node')
        
        # 1. Subscribe to the raw compressed stream
        # Check 'ros2 topic list' to ensure this name is correct
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        
        # 2. Publish the fixed RGB stream
        self.publisher_ = self.create_publisher(Image, '/image_raw_final', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info('--- IMAGE FIXER STARTING ---')
        self.get_logger().info('Repairing JPEG headers and forcing 1920 step size.')

    def listener_callback(self, msg):
        try:
            # Convert ROS byte message to numpy array
            raw_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # --- JPEG HEADER REPAIR ---
            # Some drivers strip the Start Of Image (SOI) and End Of Image (EOI) markers
            # SOI = 0xFF 0xD8 | EOI = 0xFF 0xD9
            start_marker = np.array([0xFF, 0xD8], dtype=np.uint8)
            end_marker = np.array([0xFF, 0xD9], dtype=np.uint8)
            
            # Reconstruct the JPEG file in memory
            repaired_buffer = np.concatenate((start_marker, raw_data, end_marker))
            
            # Decode the repaired buffer into a BGR image
            cv_image = cv2.imdecode(repaired_buffer, cv2.IMREAD_COLOR)

            if cv_image is not None:
                # Ensure the image is exactly 640x360
                if cv_image.shape[1] != 640 or cv_image.shape[0] != 360:
                    cv_image = cv2.resize(cv_image, (640, 360))
                
                # Convert to ROS Image message
                # This automatically sets 'step' to 1920 (640 pixels * 3 channels)
                out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                out_msg.header = msg.header
                self.publisher_.publish(out_msg)
            else:
                self.get_logger().error('JPEG Repair Failed: Decoder returned None. Data may be corrupted.')

        except Exception as e:
            self.get_logger().error(f'Critical Error in Fixer: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping Fixer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
