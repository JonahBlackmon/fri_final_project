#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import cv2
import time
from recognition_node.gesture_recognition import HandGestureRecognizer

# If your class isn't split out yet, move it to its own module or keep it in the same file

class PalmDetectionNode(Node):
    def __init__(self):
        super().__init__('palm_detection_node')
        self.publisher_ = self.create_publisher(Bool, 'palm_detected', 10)
        self.recognizer = HandGestureRecognizer()
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1, self.process_frame)  # Run at 10Hz

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read from camera.")
            return
        
        processed_frame, frame_data = self.recognizer.process_frame(frame)

        # Publish only if a palm (open hand) is detected
        if frame_data["is_hand_open"]:
            self.publisher_.publish(Bool(data=True))
            self.get_logger().info("Palm detected and published!")

        # For visualization during testing
        cv2.imshow("Palm Detection", processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PalmDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
