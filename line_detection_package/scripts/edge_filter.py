import cv2
import numpy as np


def detect_edges(image, low_threshold, high_threshold):
    
    edges=cv2.Canny(image, low_threshold, high_threshold)
    return edges

