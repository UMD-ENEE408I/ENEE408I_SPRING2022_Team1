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
    cv.imshow("3 gray_img", gray_img)

    clean_gray_img = cv.fastNlMeansDenoising(gray_img, None, 90, 7, 21)
    cv.imshow("4 clean_gray_img", clean_gray_img)

    ret2, thresh_mask = cv.threshold(clean_gray_img, 100, 255, cv.THRESH_BINARY)
    cv.imshow('5 binary thresh feed', thresh_mask)


    top_crop = thresh_mask[0:40, :] #maybe get more rows.
    left_crop = thresh_mask[:, 40:90]
    right_crop = thresh_mask[:, 550:600]

    #FOR DEBUGGING AND FINDING GOOD THRESHOLD VALUES
    ##############################################################
    #For  0:30, 110:160 and 480:530 ->
    cv.imshow('top cropped feed', top_crop)
    cv.imshow('left cropped feed', left_crop)
    cv.imshow('right cropped feed', right_crop)
    print(top_crop.sum())
    print(left_crop.sum())
    print(right_crop.sum())
    print(thresh_mask.sum())
    # time.sleep(.002)
    ###############################################################

    top_crop_sum = top_crop.sum()
    left_crop_sum = left_crop.sum()
    right_crop_sum = right_crop.sum()
    mask_sum = thresh_mask.sum()

    topThreshold = 300000
    LRThreshold = 400000
    winThreshold = 27500000

    if mask_sum > winThreshold:  # WE ARE AT MIDDLE
        return (intersectionType.Middle_of_Maze)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold and right_crop_sum > LRThreshold:  # Top Left and Right
        return (intersectionType.Three_Way)
    elif left_crop_sum > LRThreshold and right_crop_sum > LRThreshold:  # Left and Right
        return (intersectionType.T)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold:  # Top and Left
        return (intersectionType.Left_and_Forward)
    elif top_crop_sum > topThreshold and right_crop_sum > LRThreshold:  # Top and Right
        return (intersectionType.Right_and_Forward)
    elif left_crop_sum > LRThreshold:  # Left
        return (intersectionType.Left)
    elif right_crop_sum > LRThreshold:  # Right
        return (intersectionType.Right)
    else:
        return (intersectionType.Dead_End)  # It is a dead end






cap = cv.VideoCapture(0)
while True:


    ret, img = cap.read()
    cv.imshow("1 pure feed", img)

    img = img[0:380, :]

    newimg = decrease_brightness(img, 160)
    cv.imshow('2 pure feed after slice and with brightness turned down', newimg)


    type_of_inter = find_type_of_intersection(newimg) # For debug
    print(type_of_inter.name) # For debug



    if cv.waitKey(1) == 13:
        break

cap.release()
cv.destroyAllWindows()
