import cv2
import numpy as np

def convert_hls(image):
 hlsimage= cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
 return hlsimage
