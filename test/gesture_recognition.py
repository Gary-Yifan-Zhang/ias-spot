import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe Hands and Drawing modules
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Define a function to recognize basic gestures based on hand landmarks
def recognize_gesture(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    thumb_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
    
    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    middle_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    ring_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]

    # Conditions for Pointing Gesture (Index finger extended, others curled)
    if (index_finger_tip.y < index_finger_pip.y and  # Index finger is extended
        middle_finger_tip.y > middle_finger_pip.y and  # Other fingers are curled
        ring_finger_tip.y > ring_finger_pip.y and
        pinky_tip.y > pinky_pip.y):
        return "Pointing", (index_finger_tip.x, index_finger_tip.y)

    # Conditions for Fist (All fingers curled)
    if (thumb_tip.y > thumb_ip.y and  # Thumb is curled
        index_finger_tip.y > index_finger_pip.y and  # All fingers curled
        middle_finger_tip.y > middle_finger_pip.y and
        ring_finger_tip.y > ring_finger_pip.y and
        pinky_tip.y > pinky_pip.y):
        return "Fist", None
    
    # Conditions for Open Hand (All fingers extended)
    if (thumb_tip.y < thumb_ip.y and  # Thumb is extended
        index_finger_tip.y < index_finger_pip.y and  # All fingers extended
        middle_finger_tip.y < middle_finger_pip.y and
        ring_finger_tip.y < ring_finger_pip.y and
        pinky_tip.y < pinky_pip.y):
        return "Open Hand", None

    return "Other Gesture", None

# Define a function to calculate the palm center
def calculate_palm_center(hand_landmarks, image_width, image_height):
    # Using wrist, MCPs and thumb CMC to approximate the palm center
    palm_landmarks = [
        hand_landmarks.landmark[mp_hands.HandLandmark.WRIST],
        hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC],
        hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP],
    ]
    
    # Calculate the average x and y positions of the palm landmarks
    avg_x = int(np.mean([landmark.x * image_width for landmark in palm_landmarks]))
    avg_y = int(np.mean([landmark.y * image_height for landmark in palm_landmarks]))

    return (avg_x, avg_y)

# Open webcam
cap = cv2.VideoCapture(0)

# Initialize MediaPipe Hands with model complexity (for better accuracy)
with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        
        # Flip the frame horizontally for a later selfie-view display
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_height, image_width, _ = frame.shape
        
        # Process the frame with MediaPipe Hands
        results = hands.process(frame_rgb)
        
        # If hand landmarks are detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw the hand landmarks on the frame
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Recognize gesture
                gesture, fingertip_coordinates = recognize_gesture(hand_landmarks)
                
                # Calculate palm center
                palm_center = calculate_palm_center(hand_landmarks, image_width, image_height)
                
                # Display gesture text on the frame
                cv2.putText(frame, gesture, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
                # Draw the palm center on the frame
                cv2.circle(frame, palm_center, 10, (255, 0, 0), -1)  # Blue dot for palm center
                cv2.putText(frame, f"Palm Center: {palm_center}", (palm_center[0], palm_center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

                # If the gesture is "Pointing", display and return the fingertip coordinates
                if gesture == "Pointing":
                    fingertip_x = int(fingertip_coordinates[0] * image_width)
                    fingertip_y = int(fingertip_coordinates[1] * image_height)
                    cv2.circle(frame, (fingertip_x, fingertip_y), 10, (0, 255, 255), -1)  # Yellow dot for finger tip
                    cv2.putText(frame, f"Finger Tip: {(fingertip_x, fingertip_y)}", (fingertip_x, fingertip_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
                    print(f"Pointing Finger Tip: {(fingertip_x, fingertip_y)}")
        
        # Display the output frame
        cv2.imshow('MediaPipe Hands Gesture & Palm Center Recognition', frame)
        
        # Break the loop with 'q'
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

