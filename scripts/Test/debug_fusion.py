#!/usr/bin/env python
import rospy, os, sys
import time
import math

# numpy and OpenCV
import cv2
import cv2.aruco as aruco
import numpy as np
#import tf
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  source ~/tensorflow/bin/active")
    sys.exit(1)

# library to get image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# library use pose mensages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

    
###############################################################################
 
class hybrid_img:
 
  def __init__(self):

    #-- Create a publisher
    self.image_pub = rospy.Publisher("kalman/image_hybrid",Image, queue_size=1)

    #-- Create a supscriber from topic "image_raw" and publisher to "bebop/image_aruco"
    self.bridge = CvBridge()
    
    rospy.Subscriber("/bebop/image_raw", Image, self.callbackImage, queue_size=1)
    #self.image_sub = rospy.Subscriber("image", Image, self.callbackImage, queue_size=1)
    
    rospy.Subscriber('kalman/hybrid', Vector3, self.callbackHybrid, queue_size=1)
    rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN, queue_size = 1)
    #rospy.Subscriber("aruco_double/pose",Pose, self.callbackPoseAruco, queue_size = 1)

    self.vec = Vector3()
    self.status_hybrid = False
    self.DISTANCE_FOCAL = 527 

###############################################################################

  def callbackHybrid(self, pose):

    self.vec = pose
    self.vec.x = pose.x
    self.vec.y = pose.y
    self.vec.z = pose.z
    
    if(self.status_hybrid == False):
      self.status_hybrid = True

###############################################################################

  def callbackPoseRCNN(self, data):
    # recive data
    objArray = Detection2DArray()

    # rcnn_pose
    objArray = data


###############################################################################
   
  def callbackImage(self,data):

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      (rows,cols,channels) = src_image.shape
    except CvBridgeError as e:
      print(e)

    cv2.putText(src_image, "o", (cols/2, rows/2), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

    distFocus_real = self.DISTANCE_FOCAL
    x = (distFocus_real*self.vec.x)/(self.vec.z+0.001)
    y = (distFocus_real*self.vec.y)/(self.vec.z+0.001)

    X = x+(cols/2)
    Y = y+(rows/2)
    #print(X)

    # Center coordinates  
    center_coordinates = (int(X), int(Y))
    #center_coordinates = (cols/2, rows/2) 
      
    # Radius of circle 
    radius = 15
       
    # Blue color in BGR 
    color = (0, 0, 255) 
       
    # Line thickness of 5 px 
    thickness = 5
       
    # Using cv2.circle() method 
    # Draw a circle with blue line borders of thickness of 2 px
    if(self.status_hybrid == True):
      src_image = cv2.circle(src_image, center_coordinates, radius, color, thickness)
    else:
      #print("false!")

    self.status_hybrid = False
    #cv2.imshow("Image", src_image)
    #cv2.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

###############################################################################

def main(args):
  global first_time

  ic = hybrid_img()
  #-- Name of node
  rospy.init_node('hybrid_img', log_level=rospy.DEBUG)
  
  rospy.loginfo('init_node')

  try: 
    rospy.spin()
  except rospy.ROSInterruptException:
    print("Shutting down")

  #cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)