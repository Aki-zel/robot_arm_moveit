import cv2

img=cv2.imread("src/robot_control/robot_camera_control/scripts/object.png")

while True:
    cv2.imshow("1",img)
    cv2.waitKey(1)