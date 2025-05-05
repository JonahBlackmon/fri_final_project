import cv2
import mediapipe as mp
import numpy as np
import math

class HandGestureRecognizer:
    def __init__(self):
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # MediaPipe Hands landmark indices
        # For reference: https://developers.google.com/mediapipe/solutions/vision/hand_landmarker#hand_landmark_model
        self.WRIST = 0
        self.THUMB_TIP = 4
        self.INDEX_MCP = 5  # Index finger metacarpophalangeal joint
        self.INDEX_TIP = 8
        self.MIDDLE_MCP = 9  # Middle finger metacarpophalangeal joint
        self.MIDDLE_PIP = 10  # Middle finger proximal interphalangeal joint
        self.MIDDLE_TIP = 12
        self.RING_TIP = 16
        self.PINKY_TIP = 20
        
        # These will be the base of the palm for angle calculations
        self.PALM_BASE = 0  # Wrist

    def process_frame(self, frame):
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame to detect hands
        results = self.hands.process(frame_rgb)
        
        # Initialize data for the frame
        frame_data = {
            "is_hand_open": False,
            "middle_finger_angle": None
        }
        
        # If hands are detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks on the frame
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Determine if the hand is open
                is_open = self._is_hand_open(hand_landmarks)
                
                # Calculate middle finger angle
                middle_finger_angle = self._calculate_middle_finger_angle(hand_landmarks)
                
                # Update frame data
                frame_data["is_hand_open"] = is_open
                frame_data["middle_finger_angle"] = middle_finger_angle
                
                # Add text annotations to the frame
                height, width, _ = frame.shape
                cv2.putText(frame, f"Hand Open: {is_open}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Middle Finger Angle: {middle_finger_angle:.2f} degrees", 
                            (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Draw vector showing direction of middle finger
                if middle_finger_angle is not None:
                    self._draw_middle_finger_vector(frame, hand_landmarks)
        
        return frame, frame_data
    
    def _is_hand_open(self, hand_landmarks):
        """
        Determine if the hand is open based on finger positions.
        A hand is considered open if all fingertips are extended.
        """
        landmarks = hand_landmarks.landmark
        
        # Get y-coordinates of relevant landmarks
        wrist_y = landmarks[self.WRIST].y
        index_mcp_y = landmarks[self.INDEX_MCP].y
        
        # Determine finger extension by checking if the fingertips are above their respective MCPs
        # We need to compare relative to the wrist and account for the hand orientation
        is_pointing_up = wrist_y > index_mcp_y
        
        # Check all fingers (except thumb which is handled differently)
        finger_extended = [
            # For thumb: check if tip is sufficiently to the side of the MCP
            abs(landmarks[self.THUMB_TIP].x - landmarks[1].x) > 0.05,
            
            # For other fingers: check if tips are extended
            (landmarks[self.INDEX_TIP].y < landmarks[self.INDEX_MCP].y) if is_pointing_up 
            else (landmarks[self.INDEX_TIP].y > landmarks[self.INDEX_MCP].y),
            
            (landmarks[self.MIDDLE_TIP].y < landmarks[self.MIDDLE_MCP].y) if is_pointing_up 
            else (landmarks[self.MIDDLE_TIP].y > landmarks[self.MIDDLE_MCP].y),
            
            (landmarks[self.RING_TIP].y < landmarks[9].y) if is_pointing_up 
            else (landmarks[self.RING_TIP].y > landmarks[9].y),
            
            (landmarks[self.PINKY_TIP].y < landmarks[17].y) if is_pointing_up 
            else (landmarks[self.PINKY_TIP].y > landmarks[17].y)
        ]
        
        # A hand is considered open if at least 4 fingers are extended
        return sum(finger_extended) >= 4
    
    def _calculate_middle_finger_angle(self, hand_landmarks):
        """
        Calculate the angle of the middle finger relative to the palm base.
        Returns angle in degrees, where 0 degrees points up.
        """
        landmarks = hand_landmarks.landmark
        
        # Get the coordinates of the palm base (wrist)
        palm_base_x = landmarks[self.PALM_BASE].x
        palm_base_y = landmarks[self.PALM_BASE].y
        
        # Get the coordinates of the middle finger MCP (base of middle finger)
        middle_mcp_x = landmarks[self.MIDDLE_MCP].x
        middle_mcp_y = landmarks[self.MIDDLE_MCP].y
        
        # Get the coordinates of the middle finger tip
        middle_tip_x = landmarks[self.MIDDLE_TIP].x
        middle_tip_y = landmarks[self.MIDDLE_TIP].y
        
        # Calculate the vector from palm base to middle finger tip
        vec_x = middle_tip_x - palm_base_x
        vec_y = middle_tip_y - palm_base_y
        
        # Calculate the angle in radians
        angle_rad = math.atan2(-vec_y, vec_x)  # Negative vec_y because image y-axis points down
        
        # Convert to degrees and normalize to [0, 360)
        angle_deg = (math.degrees(angle_rad) + 90) % 360
        
        return angle_deg
    
    def _draw_middle_finger_vector(self, frame, hand_landmarks):
        """Draw a line representing the direction of the middle finger."""
        height, width, _ = frame.shape
        landmarks = hand_landmarks.landmark
        
        # Get pixel coordinates of palm base and middle finger tip
        palm_base = (int(landmarks[self.PALM_BASE].x * width), 
                     int(landmarks[self.PALM_BASE].y * height))
        middle_tip = (int(landmarks[self.MIDDLE_TIP].x * width), 
                      int(landmarks[self.MIDDLE_TIP].y * height))
        
        # Draw line from palm base to middle finger tip
        cv2.line(frame, palm_base, middle_tip, (0, 0, 255), 2)
        
        # Draw an arrow to make the direction more obvious
        cv2.arrowedLine(frame, palm_base, middle_tip, (0, 255, 255), 2)


def main():
    # Initialize the hand gesture recognizer
    recognizer = HandGestureRecognizer()
    
    # Open webcam
    cap = cv2.VideoCapture(0)
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        
        # Process the frame
        processed_frame, frame_data = recognizer.process_frame(frame)
        
        # Print data (for debugging)
        print(f"Hand open: {frame_data['is_hand_open']}, "
              f"Middle finger angle: {frame_data['middle_finger_angle']}")
        
        # Display the frame
        cv2.imshow('MediaPipe Hand Gesture Recognition', processed_frame)
        
        # Exit on 'q' press
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()