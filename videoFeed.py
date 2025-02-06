import cv2
import numpy as np

# Load ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# Open camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco marker
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Get center of the marker
        for corner in corners:
            x_center = int((corner[0][0][0] + corner[0][2][0]) / 2)
            y_center = int((corner[0][0][1] + corner[0][2][1]) / 2)
            cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Landing Spot: ({x_center}, {y_center})", 
                        (x_center+10, y_center-10), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0), 2)
    
    # Show video feed
    cv2.imshow("ArUco Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
