from email.message import Message
from typing import Dict
from webbrowser import get
from intersectionType import *
import socket
import time
import select




beginFlag = False
myDict = dict()
TCP_IP = '192.168.0.18'
TCP_PORT = 80
BUFFER_SIZE = 5
MESSAGE = ""
data = ""
#s = socket.socket()

def get_message1():

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        s.settimeout(3)
        global data
        data = s.recv(BUFFER_SIZE).decode().strip()
        print("received data:", data)
    except socket.error as e:
        data = ""
        print(e)
        s.close()

    return (s,data)


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
    ret2, thresh_mask = cv.threshold(gray_img, 70, 255, cv.THRESH_BINARY)
    cv.imshow('binary thresh feed', thresh_mask)


    top_crop = thresh_mask[0:40, :] #maybe get more rows.
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
    #time.sleep(.5)
    ###############################################################

    top_crop_sum = top_crop.sum()
    left_crop_sum = left_crop.sum()
    right_crop_sum = right_crop.sum()
    mask_sum = thresh_mask.sum()

    topThreshold = 200000
    LRThreshold = 300000
    winThreshold = 10000000

    if top_crop_sum > topThreshold and left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Top Left and Right
        return (intersectionType.Three_Way)
    elif left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Left and Right
        return (intersectionType.T)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold: #Top and Left
        return (intersectionType.Left_and_Forward)
    elif top_crop_sum > topThreshold and right_crop_sum > LRThreshold: #Top and Right
        return (intersectionType.Right_and_Forward)
    elif left_crop_sum > LRThreshold: # Left
        return (intersectionType.Left)
    elif right_crop_sum > LRThreshold: # Right
        return (intersectionType.Right)
    elif mask_sum > winThreshold: # WE ARE AT MIDDLE
        return (intersectionType.Middle_of_Maze)
    else:
        return (intersectionType.Dead_End) #It is a dead end






cap = cv.VideoCapture(0)
while True:

    ret, img = cap.read() #debug
    #cv.imshow('pure feed', img) #debug
    img = img[0:380, :] #debug
    newimg = decrease_brightness(img, 190) #debug
    cv.imshow('pure feed with brightness turned down', newimg) #debug

    #GET THE WIFI MESSAGE
    (s, rec_msg) = get_message1()
    if rec_msg == "Begin":
        beginFlag = True



    if(beginFlag == True):
        # print(type(img)) #This is a <class 'numpy.ndarray'>
        # print(img.shape) # img is a numpy matrix 480 x 640 x 3
        acc = 0
        setDict(myDict)

        while acc < 100:
            ret, img = cap.read()
            img = img[0:380, :]
            newimg = decrease_brightness(img, 190)
            cv.imshow('pure feed with brightness turned down in loop', newimg)
            type_of_inter = find_type_of_intersection(newimg).name # For debug
            print(type_of_inter) # For debug

            myDict[type_of_inter] += 1
            acc += 1

        type_of_inter = max(myDict, key = myDict.get)
        print('FINAL -> ' + type_of_inter)


        #NOW WE NEED TO UPDATE MAZE STRUCTURE AND SEND COMMAND BACK TO ESP32
        if type_of_inter == "Left_and_Forward":
            MESSAGE = "Forward\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Right_and_Forward":
            MESSAGE = "Right\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "T":
            MESSAGE = "Right\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Left":
            MESSAGE = "Left\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Right":
            MESSAGE = "Right\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Three_Way":
            MESSAGE = "Right\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Dead_End":
            MESSAGE = "Dead End?\n"
            s.send(MESSAGE.encode())
            s.close()
        elif type_of_inter == "Middle_of_Maze":
            MESSAGE = "WINNER\n"
            s.send(MESSAGE.encode())
            s.close()


        beginFlag = False





















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
