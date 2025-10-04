import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import numpy as np
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/annotated/compressed',
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolo11n.pt')  # Make sure the model file is available
        self.get_logger().info('YOLOv11n detector node started.')
        self.frame_count = 0
        self.last_annotated_buffer = None
        self.skip_frames = 5  # Run inference every 5th frame

    def listener_callback(self, msg):
        self.frame_count += 1
        # Convert compressed image to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error('Failed to decode image')
            return
        if self.frame_count % self.skip_frames == 0:
            # Run YOLO inference every 5th frame
            results = self.model.predict(frame, verbose=False)
            annotated_frame = results[0].plot()
            # Encode annotated frame as compressed image
            ret, buffer = cv2.imencode('.jpg', annotated_frame)
            if not ret:
                self.get_logger().error('Failed to encode annotated image')
                return
            self.last_annotated_buffer = buffer
        else:
            # Use last annotated frame for skipped frames
            buffer = self.last_annotated_buffer
            if buffer is None:
                # If no previous frame, just encode the current frame
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    self.get_logger().error('Failed to encode fallback image')
                    return
        annotated_msg = CompressedImage()
        annotated_msg.header = Header()
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_msg.format = 'jpeg'
        annotated_msg.data = buffer.tobytes()
        self.publisher.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
