import cv2
import numpy as np



def convert_gray_scale(image):
    gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray
