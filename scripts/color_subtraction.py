import numpy as np
import cv2

#video capture
cap = cv2.VideoCapture(0)

while (1):

    #frame is the main picture we will be working on next
    _,frame=cap.read()
    
    #seperating all the rgbs. The frames will show the brightest points on their scale.
    red = np.matrix(frame[:,:,2])
    green = np.matrix(frame[:, :, 1])
    blue = np.matrix(frame[:, :, 0])
    
    #subtracting everything except red. We need to make them int16 so that negative values can be taken too.
    red_only = np.int16(red) - np.int16(green) - np.int16(blue)
    
    red_only[red_only<0]=0 #incase there is a negative value
    red_only[red_only >255] = 255 #incase there is a value more than 255
    
    #to find out the x coordinate where the center of the brightest ptch lies
    column_sums = np.matrix(np.sum(red_only, 0))
    print('column shape:',column_sums.shape[1])
    column_numbers = np.matrix(np.arange(1920))
    column_mult=np.multiply(column_sums, column_numbers)
    total = np.sum(column_mult)
    total_total = np.sum(np.sum(red_only))
    column_location = total/total_total
    
    print(column_location)
    
    #to show as image we need to make 8 bit again
    
    red_only=np.uint8(red_only)
    
    
    
    
    cv2.imshow('rgb', frame)
    cv2.imshow('red', red)
    cv2.imshow('green', green)
    cv2.imshow('blue', blue)
    cv2.imshow('Red Only', red_only)
    k = cv2.waitKey(5)
    if k==27:
        break

cv2.destroyAllWindows()
print(frame)
