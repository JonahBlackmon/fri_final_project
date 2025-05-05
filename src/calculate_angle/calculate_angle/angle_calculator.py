import cv2
import mediapipe as mp
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MiddleFingerAngleCalculator:
    def __init__(self):
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # MediaPipe hand landmark indices for middle finger tracking
        self.WRIST = 0
        self.PALM_CENTER = 9
        self.MIDDLE_MCP = 9
        self.MIDDLE_PIP = 10
        self.MIDDLE_DIP = 11
        self.MIDDLE_TIP = 12
        
        # Colors for visualization
        self.PALM_COLOR = (0, 255, 0)  # Green
        self.ANGLE_COLOR = (0, 0, 255)  # Red
        self.TEXT_COLOR = (255, 255, 255)  # White

    def process_frame(self, frame):

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        visualization = frame.copy()
        height, width, _ = frame.shape
        
        output_data = {
            "hand_detected": False,
            "is_hand_open": False,
            "middle_finger_angle": None,
            "palm_direction": None
        }
        
        # If hands are detected
        if results.multi_hand_landmarks:
            output_data["hand_detected"] = True
            
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw the hand landmarks and connections
                self.mp_drawing.draw_landmarks(
                    visualization,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Extract key landmarks
                landmarks = hand_landmarks.landmark
                
                # Check if hand is open
                is_open = self._is_hand_open(landmarks)
                output_data["is_hand_open"] = is_open
                
                # Calculate middle finger angles and palm direction
                middle_finger_angle, palm_direction = self._calculate_middle_finger_angle(landmarks)
                output_data["middle_finger_angle"] = middle_finger_angle
                output_data["palm_direction"] = palm_direction
                
                # Visualize the palm and middle finger
                self._visualize_palm_and_middle_finger(visualization, landmarks, width, height)
                
                # Draw the angle visualization
                self._visualize_angle(visualization, landmarks, middle_finger_angle, width, height)
                
                # Add text information
                cv2.putText(visualization, f"Hand Open: {is_open}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT_COLOR, 2)
                cv2.putText(visualization, f"Middle Finger Angle: {middle_finger_angle:.1f} degrees", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT_COLOR, 2)
                cv2.putText(visualization, f"Palm Direction: {palm_direction:.1f} degrees", 
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT_COLOR, 2)
        
        else:
            # No hands detected
            cv2.putText(visualization, "No hands detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT_COLOR, 2)
        
        return visualization, output_data

    def _is_hand_open(self, landmarks):

        # Define fingertip and MCP landmark indices
        fingertips = [4, 8, 12, 16, 20]  # Thumb, index, middle, ring, pinky tips
        mcps = [2, 5, 9, 13, 17]  # Corresponding MCP joints
        
        # Get wrist and middle finger MCP for reference
        wrist = landmarks[0]
        middle_mcp = landmarks[9]
        
        # Determine if the hand is pointing up or down
        is_pointing_up = wrist.y > middle_mcp.y
        
        # Count extended fingers
        extended_fingers = 0
        
        # Check each finger
        for i in range(5):
            tip = landmarks[fingertips[i]]
            mcp = landmarks[mcps[i]]
            
            # For thumb (special case)
            if i == 0:
                # Check thumb extension based on x-position relative to the index finger MCP
                if ((tip.x < landmarks[5].x) if landmarks[5].x < landmarks[17].x 
                    else (tip.x > landmarks[5].x)):
                    extended_fingers += 1
            else:
                # For other fingers, check if tip is extended further than MCP
                if (tip.y < mcp.y) if is_pointing_up else (tip.y > mcp.y):
                    extended_fingers += 1
        
        # Consider hand open if at least 4 fingers are extended
        return extended_fingers >= 4

    def _calculate_middle_finger_angle(self, landmarks):
        # Get coordinates of relevant landmarks
        wrist = (landmarks[self.WRIST].x, landmarks[self.WRIST].y)
        palm_center = (landmarks[self.PALM_CENTER].x, landmarks[self.PALM_CENTER].y)
        middle_mcp = (landmarks[self.MIDDLE_MCP].x, landmarks[self.MIDDLE_MCP].y)
        middle_pip = (landmarks[self.MIDDLE_PIP].x, landmarks[self.MIDDLE_PIP].y)
        middle_dip = (landmarks[self.MIDDLE_DIP].x, landmarks[self.MIDDLE_DIP].y)
        middle_tip = (landmarks[self.MIDDLE_TIP].x, landmarks[self.MIDDLE_TIP].y)
        
        # Calculate palm direction vector (from wrist to middle finger MCP)
        palm_vector_x = palm_center[0] - wrist[0]
        palm_vector_y = palm_center[1] - wrist[1]
        
        # Calculate palm direction angle
        palm_direction = math.degrees(math.atan2(-palm_vector_y, palm_vector_x)) + 90
        palm_direction = (palm_direction + 360) % 360  # Normalize to [0, 360)
        
        # Calculate middle finger direction vector (from MCP to tip)
        finger_vector_x = middle_tip[0] - middle_mcp[0]
        finger_vector_y = middle_tip[1] - middle_mcp[1]
        
        # Calculate middle finger direction angle
        finger_direction = math.degrees(math.atan2(-finger_vector_y, finger_vector_x)) + 90
        finger_direction = (finger_direction + 360) % 360  # Normalize to [0, 360)
        
        # Calculate the angle between palm and middle finger
        angle_diff = (finger_direction - palm_direction + 360) % 360
        # Adjust for the most intuitive angle representation
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        return angle_diff, palm_direction

    def _visualize_palm_and_middle_finger(self, frame, landmarks, width, height):
        """Draw palm and middle finger vectors."""
        # Convert normalized coordinates to pixel coordinates
        wrist = (int(landmarks[self.WRIST].x * width), int(landmarks[self.WRIST].y * height))
        palm_center = (int(landmarks[self.PALM_CENTER].x * width), int(landmarks[self.PALM_CENTER].y * height))
        middle_mcp = (int(landmarks[self.MIDDLE_MCP].x * width), int(landmarks[self.MIDDLE_MCP].y * height))
        middle_tip = (int(landmarks[self.MIDDLE_TIP].x * width), int(landmarks[self.MIDDLE_TIP].y * height))
        
        # Draw palm vector
        cv2.line(frame, wrist, palm_center, self.PALM_COLOR, 3)
        cv2.arrowedLine(frame, wrist, palm_center, self.PALM_COLOR, 5)
        
        # Draw middle finger vector
        cv2.line(frame, middle_mcp, middle_tip, self.ANGLE_COLOR, 3)
        cv2.arrowedLine(frame, middle_mcp, middle_tip, self.ANGLE_COLOR, 5)

    def _visualize_angle(self, frame, landmarks, angle, width, height):
        """Draw an arc representing the angle between palm and middle finger."""
        # Convert normalized coordinates to pixel coordinates
        wrist = (int(landmarks[self.WRIST].x * width), int(landmarks[self.WRIST].y * height))
        palm_center = (int(landmarks[self.PALM_CENTER].x * width), int(landmarks[self.PALM_CENTER].y * height))
        middle_mcp = (int(landmarks[self.MIDDLE_MCP].x * width), int(landmarks[self.MIDDLE_MCP].y * height))
        middle_tip = (int(landmarks[self.MIDDLE_TIP].x * width), int(landmarks[self.MIDDLE_TIP].y * height))
        
        # Calculate palm vector
        palm_vector_x = palm_center[0] - wrist[0]
        palm_vector_y = palm_center[1] - wrist[1]
        palm_direction = math.degrees(math.atan2(-palm_vector_y, palm_vector_x)) + 90
        palm_direction = (palm_direction + 360) % 360
        
        # Calculate middle finger vector
        finger_vector_x = middle_tip[0] - middle_mcp[0]
        finger_vector_y = middle_tip[1] - middle_mcp[1]
        finger_direction = math.degrees(math.atan2(-finger_vector_y, finger_vector_x)) + 90
        finger_direction = (finger_direction + 360) % 360
        
        # Draw an arc showing the angle
        radius = 30
        start_angle = (palm_direction - 90) % 360
        end_angle = (finger_direction - 90) % 360
        
        if abs(end_angle - start_angle) > 180:
            if start_angle < end_angle:
                start_angle += 360
            else:
                end_angle += 360
                
        start_angle_rad = math.radians(start_angle)
        end_angle_rad = math.radians(end_angle)
        
        cv2.ellipse(frame, middle_mcp, (radius, radius), 0, 
                   min(start_angle, end_angle), max(start_angle, end_angle), 
                   (255, 0, 255), 2)

        text_x = middle_mcp[0] + int(radius * 1.5 * math.cos(math.radians((start_angle + end_angle) / 2)))
        text_y = middle_mcp[1] + int(radius * 1.5 * math.sin(math.radians((start_angle + end_angle) / 2)))
        cv2.putText(frame, f"{angle:.1f}°", (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)


def main():
    rclpy.init()
    node = rclpy.create_node('middle_finger_angle_publisher')
    publisher = node.create_publisher(Float32, 'middle_finger_angle', 10)

    calculator = MiddleFingerAngleCalculator()
    cap = cv2.VideoCapture(0)

    try:
        while cap.isOpened():
            rclpy.spin_once(node, timeout_sec=0)

            success, frame = cap.read()
            if not success:
                print("Failed to capture frame.")
                continue

            visualization, data = calculator.process_frame(frame)

            if data["hand_detected"] and data["palm_direction"] is not None:

                msg = Float32()
                msg.data = float(data["palm_direction"])

                publisher.publish(msg)
                
                # Debugging
                print(f"Published palm direction: {msg.data:.1f}°")

            cv2.imshow('Middle Finger Angle', visualization)

            # Exit if the user presses 'q'
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
