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

# library use pose mensages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

#-- Define Tag\n",
id_to_find = 273 # 1 273
marker_size = 0.5 # 0.7 #-m -  0.172 m

out = True
ids = []

#-- Define the Aruco dictionary\n",
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) #pata de urso
#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) #
parameters =  aruco.DetectorParameters_create()

######### -- Get the camera calibration\n" ############

#-- Get the camera calibration\n",
CAMERA_INFO_PATH = os.path.join(os.path.dirname(sys.path[0]),'camera_info')
print(CAMERA_INFO_PATH)
camera_matrix = np.loadtxt(CAMERA_INFO_PATH + '/cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt(CAMERA_INFO_PATH + '/cameraDistortion.txt', delimiter = ',')

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN


###############################################################################
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
  #-- transpose the matrix R
  Rt = np.transpose(R)

  #-- verify if Rt could be identity
  shouldBeIdentity = np.dot(Rt, R)

  #-- create a identity
  I = np.identity(3, dtype=R.dtype)
  n = np.linalg.norm(I - shouldBeIdentity)

  return n < 1e-6

###############################################################################

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
    
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
    #rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN, queue_size = 1)

    self.Keyframe_aruco = 0
    self.vec = Vector3()

###############################################################################

  def callbackHybrid(self, pose):

    self.vec = pose
    self.vec.x = pose.x
    self.vec.y = pose.y
    self.vec.z = pose.z

###############################################################################
   
  def callbackImage(self,data):

    global out, ids
    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      (rows,cols,channels) = src_image.shape
    except CvBridgeError as e:
      print(e)

    #-- 180 deg rotation matrix around x axis
    R_flip = np.zeros((3,3), dtype=np.float)
    R_flip[0,0] = +1.0
    R_flip[1,1] = -1.0
    R_flip[2,2] = -1.0

    #-- Convert in gray scale\n",
    gray = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image\n",
    corners, ids, rejected = aruco.detectMarkers(image=gray,
                                                  dictionary=aruco_dict,
                                                  parameters=parameters,
                                                  cameraMatrix=camera_matrix,
                                                  distCoeff=camera_distortion)

    #rospy.loginfo("ids[]: %f", len(ids[0]))
    #int(ids) == id_to_find:

    # out = np.logical_and(ids != None, ids[0] == id_to_find)
    # rospy.loginfo(out)

    if ids != None:
      #rospy.loginfo("Aruco working!")

      if ids[0] == id_to_find:

        #if (int(ids[0]) != None and int(ids[0]) == id_to_find):
        #-- ret= [rvec,tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1, [rvec2], ...]]  attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1, [tvec2], ...]]  position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first\n",
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it\n",
        aruco.drawDetectedMarkers(src_image, corners)
        aruco.drawAxis(src_image, camera_matrix, camera_distortion, rvec, tvec, 0.3)

        #-- Obtain the rotation matrix tag->camera
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T # function transpose() with '.T'

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        pitch_marker, roll_marker, yaw_marker = rotationMatrixToEulerAngles(R_tc)

        #-- Now get Position and attitude f the camera respect to the marker
        #pos_camera = -R_tc*np.matrix(tvec).T
        pitch_camera, roll_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

        pos_camera = Point(-tvec[0], tvec[1], tvec[2])

        #-- Print 'X' in the center of the camera
        cv2.putText(src_image, "X", (cols/2, rows/2), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

        ###############################################################################

        #-- Print the tag position in camera frame
        str_position = "Position x = %4.0f  y = %4.0f  z = %4.0f"%(pos_camera.x, pos_camera.y, pos_camera.z)
        cv2.putText(src_image, str_position, (0, 30), font, 2, (255, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        str_attitude = "Attitude pitch = %4.0f  roll = %4.0f  yaw = %4.0f"%(math.degrees(0),math.degrees(0),
                            math.degrees(yaw_camera))
        cv2.putText(src_image, str_attitude, (0, 60), font, 2, (255, 255, 0), 2, cv2.LINE_AA)

        ###############################################################################

        #rospy.loginfo('Id detected!')

    #else:
      #rospy.loginfo('No Id detected!')
      #print('No Id detected!')

    # Center coordinates 
    center_coordinates = (cols*self.vec.x, rows*self.vec.y) 
      
    # Radius of circle 
    radius = 20
       
    # Blue color in BGR 
    color = (255, 0, 0) 
       
    # Line thickness of 3 px 
    thickness = 3
       
    # Using cv2.circle() method 
    # Draw a circle with blue line borders of thickness of 2 px 
    src_image = cv2.circle(src_image, center_coordinates, radius, color, thickness) 
    
    cv2.imshow("Image", src_image)
    cv2.waitKey(1)

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

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)