import cv2
import numpy as np



def apply_smoothing(image, kernel_size):
    """
    kernel_size must be postivie and odd
    
    """
    
    
    gauss=cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    return gauss
