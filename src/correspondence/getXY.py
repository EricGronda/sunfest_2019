# File: getXY.py
# Taken from: https://aspratyush.wordpress.com/2013/12/22/ginput-in-opencv-python/
# Modified by: Eric Gronda (eric.gronda@umbc.edu)
# Date: 7/24/19
# Description:
#    This code mimics the effect of matlab's ginput in opencv-python.
#    It has been modified from the original to directly receive a 
#    cv image, instead of an image path
import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
 
a = np.array([0,0], dtype='float32')
def getXY(img):

 #define the event
 def getxy(event, x, y, flags, param):
   global a
   if event == cv2.EVENT_LBUTTONDOWN :
      a = np.vstack([a, np.hstack([x,y])])
      
      if (True):
         print "(row, col) = ", (x,y)

 #Set mouse CallBack event
 cv2.namedWindow('image')
 cv2.setMouseCallback('image', getxy)

 #show the image
 print "Click to select a point OR press ANY KEY to continue..."
 cv2.imshow('image', img)
 cv2.waitKey(0)
 cv2.destroyAllWindows()
 
 #obtain the matrix of the selected points
 b = a[1:,:]
 return b
