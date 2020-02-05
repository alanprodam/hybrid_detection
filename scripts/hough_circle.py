#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys, time, math
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

class hough_circle:
 
  def __init__(self):
    #-- Create a publisher in topic "image_hough" and "nav_hough_lines"
    self.pose_circle_pub = rospy.Publisher("hough/pose_circle",Vector3, queue_size = 1)
    
    #self.image_raw_pub = rospy.Publisher("hough/image_raw/compressed", CompressedImage, queue_size = 1)
    self.image_hough_pub = rospy.Publisher("hough/image_hough", Image, queue_size = 1)    
    #self.image_edge_pub = rospy.Publisher("hough/image_edge", Image, queue_size = 1)

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw", Image, self.callback, queue_size = 1)

    self.MIN_EDGE = 100
    self.MAX_EDGE = 150

###############################################################################
  def callback(self,data):


    try:
        image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    # image_np = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

    # (rows,cols,channels) = image_np.shape
    # rospy.logdebug("rows: %f",rows)
    # rospy.logdebug("cols: %f",cols)
    # rospy.logdebug("-------------------------")

    #-- Resize image with INTER_CUBIC
    #resize = cv2.resize(image_np, (224, 224), interpolation=cv2.INTER_CUBIC)

    #-- Convert in gray scale
    gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Detection de edges
    edges = cv2.Canny(gray, self.MIN_EDGE, self.MAX_EDGE, apertureSize=3, L2gradient=True) # default (350,400)

    #-- Blur bilateral filter
    blur = cv2.bilateralFilter(edges,3,75,75)
    #blur2 = cv2.GaussianBlur(edges,(5,5),0)

    #-- Erosion and Dilation
    kernel_dil = np.ones((5,5), np.uint8)
    kernel_ero = np.ones((3,3), np.uint8)

    dilation = cv2.dilate(blur, kernel_dil, iterations=1)
    erosion = cv2.erode(dilation, kernel_ero, iterations=1) 


    # Otsu's thresholding after Gaussian filtering
    #gray2 = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
  
    #ret1, th1 = cv2.threshold(gray2,100,150,cv2.THRESH_BINARY)
    #ret2, th2 = cv2.threshold(gray2,50,100,cv2.THRESH_OTSU)
    #blur2 = cv2.GaussianBlur(gray2,(5,5),0)
    #ret3, th3 = cv2.threshold(blur2,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #Deteccao de linhas
    #lines = cv2.HoughLines(erosion, numLines, np.pi/90, 100)

    

    # raw_image = cv2.imread('rawImage.jpg')
    # cv2.imshow('Original Image', raw_image)
    # cv2.waitKey(0)

    # bilateral_filtered_image = cv2.bilateralFilter(raw_image, 5, 175, 175)
    # cv2.imshow('Bilateral', bilateral_filtered_image)
    # cv2.waitKey(0)

    # edge_detected_image = cv2.Canny(bilateral_filtered_image, 75, 200)
    # cv2.imshow('Edge', edge_detected_image)
    # cv2.waitKey(0)

    # _, contours, hierarchy = cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # contour_list = []
    # for contour in contours:
    #     approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
    #     area = cv2.contourArea(contour)
    #     if ((len(approx) > 8) & (len(approx) < 23) & (area > 30) ):
    #         contour_list.append(contour)

    # cv2.drawContours(raw_image, contour_list,  -1, (255,0,0), 2)
    # cv2.imshow('Objects Detected',raw_image)
    # cv2.waitKey(0)
  

    # Filter yaw
    # if len(self.list_hough) < 1:
    #     self.list_hough.append(yaw)
        
    # else:
    #     self.list_hough.append(yaw)
    #     del self.list_hough[0]
    #     yaw_filtered = sum(self.list_hough)/len(self.list_hough)
    #     rospy.logdebug('Size of list_hough %f',len(self.list_hough))
    #     rospy.logdebug('Yaw (Filter): %f deg/s',yaw_filtered)
    #     rospy.logdebug("-------------------------")


    pose_landmark = Vector3()

    if lines is not None:
      pose_landmark.x = 0
      pose_landmark.y = 0
      pose_landmark.z = 0

    else:
      pose_landmark.x = 0
      pose_landmark.y = 0
      pose_landmark.z = 0


    # rospy.logdebug('Yaw Raw: %f deg/s',yaw)
    # rospy.logdebug("-------------------------")

    try:
      self.pose_circle_pub.publish(pose_landmark)
      #rospy.logdebug('Is publish!')
    except:
      rospy.logdebug('No publish lines!')

    #cv2.imshow("Image",src_image)
    #cv2.imshow("Image-edges",edges)
    
    #cv2.imshow("Image-blur",blur)

    #cv2.imshow("Image-dilation",dilation)
    #cv2.imshow("Image-erosion",erosion)
    #cv2.waitKey(1)

    try:
        self.image_hough_pub.publish(self.bridge.cv2_to_imgmsg(image_np, "bgr8"))
        #self.image_edge_pub.publish(self.bridge.cv2_to_imgmsg(erosion, "mono8"))
    except CvBridgeError as e:
        print(e)
        rospy.logdebug('No publish img!')

###############################################################################

def main(args):

  ic = hough_circle()
  #-- Name of node
  rospy.init_node('hough_circle',log_level=rospy.DEBUG)

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)
