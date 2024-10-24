import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO Pins for Motor Control
MOTOR_LEFT_FORWARD = 17
MOTOR_LEFT_BACKWARD = 27
MOTOR_RIGHT_FORWARD = 22
MOTOR_RIGHT_BACKWARD = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)

# Motor control functions
def stop():
    GPIO.output(MOTOR_LEFT_FORWARD, False)
    GPIO.output(MOTOR_LEFT_BACKWARD, False)
    GPIO.output(MOTOR_RIGHT_FORWARD, False)
    GPIO.output(MOTOR_RIGHT_BACKWARD, False)

def move_forward():
    GPIO.output(MOTOR_LEFT_FORWARD, True)
    GPIO.output(MOTOR_LEFT_BACKWARD, False)
    GPIO.output(MOTOR_RIGHT_FORWARD, True)
    GPIO.output(MOTOR_RIGHT_BACKWARD, False)

def turn_left():
    GPIO.output(MOTOR_LEFT_FORWARD, False)
    GPIO.output(MOTOR_LEFT_BACKWARD, True)
    GPIO.output(MOTOR_RIGHT_FORWARD, True)
    GPIO.output(MOTOR_RIGHT_BACKWARD, False)

def turn_right():
    GPIO.output(MOTOR_LEFT_FORWARD, True)
    GPIO.output(MOTOR_LEFT_BACKWARD, False)
    GPIO.output(MOTOR_RIGHT_FORWARD, False)
    GPIO.output(MOTOR_RIGHT_BACKWARD, True)

# OpenCV Setup for Camera
cap = cv2.VideoCapture(0)

def detect_obstacle(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # GaussianBlur to reduce noise and improve detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection using Canny
    edges = cv2.Canny(blurred, 50, 150)

    # Define region of interest (only the bottom half of the image)
    height, width = frame.shape[:2]
    mask = np.zeros_like(edges)
    mask[int(height/2):, :] = 255

    # Apply the mask
    masked_edges = cv2.bitwise_and(edges, mask)

    # Find contours
    contours, _ = cv2.findContours(masked_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # If no contours found, return no obstacle
    if len(contours) == 0:
        return None

    # Get the largest contour (assumed to be the obstacle)
    obstacle_contour = max(contours, key=cv2.contourArea)

    # Get bounding rectangle for the contour
    x, y, w, h = cv2.boundingRect(obstacle_contour)
    return (x, y, w, h)

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame")
            break

        # Resize the frame for faster processing
        frame = cv2.resize(frame, (320, 240))

        # Detect obstacle
        obstacle = detect_obstacle(frame)

        if obstacle:
            x, y, w, h = obstacle

            # Draw a rectangle around the detected obstacle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Divide the screen into 3 zones: left, center, right
            screen_middle = frame.shape[1] // 2
            object_center = x + w // 2

            if object_center < screen_middle - 50:
                print("Obstacle on the left, turning right")
                turn_right()
                time.sleep(0.5)  # Adjust this to change the speed of the turn
            elif object_center > screen_middle + 50:
                print("Obstacle on the right, turning left")
                turn_left()
                time.sleep(0.5)
            else:
                print("Obstacle in front, stopping")
                stop()
                time.sleep(1)  # Wait for a second
        else:
            print("No obstacle detected, moving forward")
            move_forward()

        # Display the frame (optional for debugging)
        cv2.imshow('Frame', frame)

        # Exit the loop on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
