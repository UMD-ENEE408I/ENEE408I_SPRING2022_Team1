import cv2 as cv
import numpy


def lane_cheeck(thresh_mask):
    return 0


cap = cv.VideoCapture(0)
while True:
    ret, img = cap.read()

    # print(type(img)) #This is a <class 'numpy.ndarray'>
    # print(img.shape) # img is a numpy matrix 480 x 640 x 3
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret2, thresh_mask = cv.threshold(gray_img, 210, 255, cv.THRESH_BINARY)
    cv.imshow('binary thresh feed', thresh_mask)
    # print(gray_img[:, 30])

    lane_cheeck(thresh_mask)

    ############################################################################
    ##################Otsu's THRESHOLDING########################################
    #####################################################################################
    # Otsu's thresholding
    # ret2, th2 = cv.threshold(gray_img, 205, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    # cv.imshow('otsu thresh feed', th2)

    # Otsu's thresholding after Gaussian filtering
    # blur = cv.GaussianBlur(gray_img, (9, 9), 0)
    # ret3, th3 = cv.threshold(blur, 205, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    # cv.imshow('gauss and otsu thresh feed', th3)

    ############################################################################
    ##################ADAPTIVE THRESHOLDING########################################
    #####################################################################################
    # th4 = cv.adaptiveThreshold(gray_img, 30, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 3, 2)
    # cv.imshow('adapt thresh 1 feed', th4)

    # th5 = cv.adaptiveThreshold(gray_img, 30, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 3, 2)
    # cv.imshow('adapt thresh 2 feed', th5)

    ############################################################################
    ##################CANNY EDGE DETECTION########################################
    ######################################################################################
    # blur = cv.GaussianBlur(gray_img, (5, 5), 0)
    # canny = cv.Canny(blur, 20, 70)
    # ret, mask = cv.threshold(canny, 240, 255, cv.THRESH_BINARY)
    # cv2.imshow('canny feed', mask)

    if cv.waitKey(1) == 13:
        break

cap.release()
cv.destroyAllWindows()
