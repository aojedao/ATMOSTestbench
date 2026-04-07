import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera_node')
        
        # Publishers - topic names matching your Aruco node
        self.img_pub = self.create_publisher(Image, '/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.bridge = CvBridge()
        
        # Robust GStreamer Pipeline
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=640, height=360, framerate=30/1 ! "
            "jpegdec ! videoconvert ! appsink drop=true"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('GStreamer failed to open! Check /dev/video0')
            exit()

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('Camera and Info Node Started Successfully.')

    def timer_callback(self):
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return

            # Get current time for sync
            now = self.get_clock().now().to_msg()
            
            # 1. Publish Image
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera_link"
            self.img_pub.publish(img_msg)
            
            # 2. Construct and Publish CameraInfo (Strict Float Lists)
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = "camera_link"
            info_msg.width = 640
            info_msg.height = 360
            info_msg.distortion_model = "plumb_bob"
            
            # Explicitly cast everything to float to avoid DDS serialization errors
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 180.0, 0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            
            self.info_pub.publish(info_msg)

        except Exception as e:
            # This will finally show us why the video is stopping!
            self.get_logger().error(f"Timer Callback Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
