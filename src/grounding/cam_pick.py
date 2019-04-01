import cv2
import numpy as np

cap = cv2.VideoCapture(0)
# cap.set(3, 1280)
# cap.set(4, 720)
# cap.set(17, 3600)
path = "/home/clarence/Desktop/Video/edge/"
counter = 0

while(1):
    # get a frame
    ret, frame = cap.read()
    # show a frame
    cv2.imshow("capture", frame)
    # cv2.imwrite(path + str(counter) + ".png", frame)
    counter += 1

    if cv2.waitKey(100) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows() 
        break


cap.release()
cv2.destroyAllWindows() 
