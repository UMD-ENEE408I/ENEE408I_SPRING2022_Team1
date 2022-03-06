import cv2
import numpy

cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()


    cv2.imshow('Video feed', img)

    if cv2.waitKey(1) == 13:
        break
cap.release()
cv2.destroyAllWindows()