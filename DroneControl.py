import cv2
import numpy as np
from djitellopy import Tello
import time

# Initialize Tello drone
tello = Tello()
tello.connect()
tello.streamon()

# Load ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# PID control variables
Kp = 0.4  # Proportional gain
Ki = 0.01 # Integral gain
Kd = 0.2  # Derivative gain

previous_error_x = 0
previous_error_y = 0
integral_x = 0
integral_y = 0

# Get video feed
frame_read = tello.get_frame_read()

# Take off
tello.takeoff()
time.sleep(2)

try:
    while True:
        frame = frame_read.frame
        frame = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for corner in corners:
                x_center = int((corner[0][0][0] + corner[0][2][0]) / 2)
                y_center = int((corner[0][0][1] + corner[0][2][1]) / 2)

                # Compute error (distance from center of frame)
                error_x = x_center - 320  # 640/2 (center of the frame)
                error_y = y_center - 240  # 480/2

                # PID calculations
                integral_x += error_x
                integral_y += error_y
                derivative_x = error_x - previous_error_x
                derivative_y = error_y - previous_error_y
                
                move_x = int(Kp * error_x + Ki * integral_x + Kd * derivative_x)
                move_y = int(Kp * error_y + Ki * integral_y + Kd * derivative_y)

                previous_error_x = error_x
                previous_error_y = error_y

                # Adjust drone position
                if abs(error_x) > 20:
                    tello.send_rc_control(-move_x // 10, 0, 0, 0)
                if abs(error_y) > 20:
                    tello.send_rc_control(0, 0, -move_y // 10, 0)

                # If close to center, land
                if abs(error_x) < 20 and abs(error_y) < 20:
                    tello.land()
                    break

        cv2.imshow("Tello ArUco Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    tello.land()
    tello.streamoff()
    tello.end()
    cv2.destroyAllWindows()
