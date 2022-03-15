from typing import Dict
import cv2 as cv
import numpy as np
import time
from intersectionType import *

beginFlag = True
myDict = dict()






#Idea should be to get the binary mask
#get rid of as much noise as possible and keep the line
#we will leave lane checking to stay on track for the light bar
#now we need to create protocol for intersection detection
#
#when the beginFlag is made true in the while loop, we can assume the arduino/platformIO code has the mouse 
#positioned correctly 

def find_type_of_intersection(img):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    #blur = cv.GaussianBlur(gray_img, (5, 5), 0)
    ret2, thresh_mask = cv.threshold(gray_img, 205, 255, cv.THRESH_BINARY)
    cv.imshow('binary thresh feed', thresh_mask)


    top_crop = thresh_mask[0:30, :] #maybe get more rows.
    left_crop = thresh_mask[:, 110:160]
    right_crop = thresh_mask[:, 480:530]

    #FOR DEBUGGING AND FINDING GOOD THRESHOLD VALUES
    ##############################################################
    #For  0:30, 110:160 and 480:530 ->  80000 seems to work
    cv.imshow('top cropped feed', top_crop)
    cv.imshow('left cropped feed', left_crop)
    cv.imshow('right cropped feed', right_crop)
    print(top_crop.sum())
    print(left_crop.sum())
    print(right_crop.sum())
    time.sleep(.5)  
    ###############################################################

    top_crop_sum = top_crop.sum()
    left_crop_sum = left_crop.sum()
    right_crop_sum = right_crop.sum()
    topThreshold = 200000
    LRThreshold = 200000

    if top_crop_sum > topThreshold and left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Top Left and Right
        return (intersectionType.Three_Way)
    elif left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Left and Right
        return (intersectionType.T)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold: #Top and Left
        return (intersectionType.Left_and_Foward)
    elif top_crop_sum > topThreshold and right_crop_sum > LRThreshold: #Top and Right
        return (intersectionType.Right_and_Foward)
    elif left_crop_sum > LRThreshold: # Left
        return (intersectionType.Left)
    else: # Right
        return (intersectionType.Right)






cap = cv.VideoCapture(1)
while True:

    ret, img = cap.read()
    cv.imshow('pure feed', img)

    type_of_inter = find_type_of_intersection(img) # For debug
    print(type_of_inter.name) # For debug





















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
