import cv2
import numpy

cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    #print(type(img)) #This is a <class 'numpy.ndarray'>
    print(img.shape) # img is a numpy matrix 480 x 640 x 3


    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img, (7, 7), 0)
    canny = cv2.Canny(blur, 20, 70)
    ret, mask = cv2.threshold(canny, 70, 255, cv2.THRESH_BINARY)
    cv2.imshow('Video feed', mask)





    if cv2.waitKey(1) == 13:
        break


cap.release()
cv2.destroyAllWindows()