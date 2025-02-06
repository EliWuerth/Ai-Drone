import cv2
import numpy as np

# Initialize the video capture
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

# Define the range of the target color in HSV
lower_color = np.array([30, 150, 50])  # Example for green color
upper_color = np.array([80, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the target color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours of the target
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw a rectangle around the target
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Here you would add code to control the drone to follow the target
        # For example, adjust the drone's position based on the target's location

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()