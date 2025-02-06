import cv2
import numpy as np

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
marker_size = 200  # Size in pixels

marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
marker_image = cv2.aruco.drawMarker(aruco_dict, 0, marker_size)

cv2.imwrite("aruco_marker.png", marker_image)
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
