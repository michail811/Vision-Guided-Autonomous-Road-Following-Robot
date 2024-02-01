import cv2
import numpy as np

def white(image):
     #white color mask
    lower = np.uint8([  0, 80,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(image, lower, upper)
    
    return white_mask