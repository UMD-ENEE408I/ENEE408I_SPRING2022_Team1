import cv2 as cv
import numpy


#Idea should be to get the binary mask
#get rid of as much noise as possible and keep the line
#we will leave lane checking to stay on track for the light bar
#now we need to create protocol for intersection detection
#
#After getting the binary mask, we will take a 30x640 row vectors at the top of the matrix,
#we take the sum of this matrix and once it passes a threshold we send a stop and push by some constant command to esp32
#so we can align up the camera with the entire intersection.
#We wait for esp32 complete response and then find what type of intersection we have.
#now ....

def find_type_of_intersection(thresh_mask):
    top_crop = thresh_mask[0:30, :] #maybe get more rows.
    #if sum(top_crop) > somenumber:
    #   send stop and push command
    #
    #
    #
    #

cap = cv.VideoCapture(0)
while True:
    ret, img = cap.read()

    # print(type(img)) #This is a <class 'numpy.ndarray'>
    # print(img.shape) # img is a numpy matrix 480 x 640 x 3
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray_img, (5, 5), 0)
    ret2, thresh_mask = cv.threshold(blur, 210, 255, cv.THRESH_BINARY)
    cv.imshow('binary thresh feed', thresh_mask)
    # print(gray_img[:, 30])



    find_type_of_intersection(thresh_mask)



















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
    ##################CANNY########################################
    #####################################################################################
    #blur = cv.GaussianBlur(gray_img, (5, 5), 0)
    #canny = cv.Canny(blur, 10, 70)
    #ret, mask = cv.threshold(canny, 70, 255, cv.THRESH_BINARY)
    #cv.imshow('canny feed', mask)




    if cv.waitKey(1) == 13:
        break

cap.release()
cv.destroyAllWindows()
