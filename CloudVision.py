import time

import numpy as np
from filterpy.kalman import KalmanFilter

class Drone:
    def __init__(self):
        self.position = (0, 0)  # x, y coordinates
        self.target_position = (0, 0)

    def update_target(self, target_position):
        self.target_position = target_position

    def move_towards_target(self):
        # Simple logic to move towards the target
        if self.position[0] < self.target_position[0]:
            self.position = (self.position[0] + 1, self.position[1])
        elif self.position[0] > self.target_position[0]:
            self.position = (self.position[0] - 1, self.position[1])

        if self.position[1] < self.target_position[1]:
            self.position = (self.position[0], self.position[1] + 1)
        elif self.position[1] > self.target_position[1]:
            self.position = (self.position[0], self.position[1] - 1)

        print(f"Drone moving to position: {self.position}")

# Example usage 
drone = Drone()
drone.update_target((5, 5))  # Set target position
while drone.position != drone.target_position:
    drone.move_towards_target()
    time.sleep(1)  # Simulate time delay for movement


# Initialize Kalman Filter
kf = KalmanFilter(dim_x=4, dim_z=2)
kf.x = np.array([0, 0, 0, 0])  # Initial state (x position, y position, x velocity, y velocity)
kf.P *= 1000.  # Initial uncertainty
kf.R = np.array([[5, 0], [0, 5]])  # Measurement noise
kf.Q = np.eye(4)  # Process noise

def update_kalman_filter(measurement):
    kf.predict()  # Predict the next state
    kf.update(measurement)  # Update with the new measurement
    return kf.x  # Return the updated state

# Example measurement from GPS (x, y)
gps_measurement = np.array([10, 20])
updated_state = update_kalman_filter(gps_measurement)

print("Updated State:", updated_state)