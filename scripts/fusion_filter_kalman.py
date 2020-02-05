#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np
import math
import tf

# ROS related imports of Odometry ans Navigation
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, Vector3

# ROS related imports of Vision
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class Kalman(object):
    """docstring for Kalman"""
    def __init__(self, n_states, n_sensors):
        super(Kalman, self).__init__()
        self.n_states = n_states
        self.n_sensors = n_sensors

        self.I = np.matrix(np.identity(n_states))                   #identity(3,3)
        self.x = np.matrix(np.zeros(shape=(n_states,1)))            #zeros(3,1)
        self.P = np.matrix(np.identity(n_states))                   #identity(3,3)                  
        self.F = np.matrix(np.identity(n_states))                   #identity(3,3)  
        self.u = np.matrix(np.zeros(shape=(n_states,1)))            #zeros(3,1)
        self.H = np.concatenate((self.I,self.I), axis=0)            #identity(3,3)
        self.R = np.matrix(np.identity(n_sensors))                  #identity(6,6)
        self.Q = np.matrix(np.identity(n_states))                   #identity(3,3) 

        self.first = True

    def update(self, Z):

        w = Z - self.H * self.x
        S = self.H * self.P * self.H.getT() + self.R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x # + self.u
        self.P = self.F * self.P * self.F.getT() + self.Q

class Subscriber(object):
    
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('filter_node', anonymous=True, log_level=rospy.DEBUG)

        self.kalman = Kalman(n_states = 3, n_sensors = 6)
        self.kalman.P *= 10
        self.kalman.R *= 0.02

        self.VecNeural = Vector3()
        self.VecAruco = Vector3()
        self.OriAruco = Quaternion(0,0,0,1)

        self.list_z = []
        self.VecNeural_z_ant = 0

        # Publishers
        self.pub_hibrid = rospy.Publisher('kalman/hybrid', Vector3)
        self.odom_filter_pub = rospy.Publisher("odom_filter", Odometry)
        self.odom_rcnn_pub = rospy.Publisher("odom_rcnn", Odometry)

        # transform tf
        tf_hybrid_to_drone = tf.TransformBroadcaster()
        tf_rcnn_to_drone = tf.TransformBroadcaster()

        Keyframe = 0

        rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        rospy.Subscriber("aruco_double/pose",Pose, self.callbackPoseAruco)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            neural = [self.VecNeural.x, self.VecNeural.y, self.VecNeural.z]
            aruco = [self.VecAruco.x, self.VecAruco.y, self.VecAruco.z]

            mat = np.matrix(np.concatenate((neural, aruco), axis=None)).getT()

            if self.kalman.first:
                # insert the first 3 values of the vector 
                self.kalman.x = mat[0:3, :]
                self.kalman.first = False

            module_aru = math.sqrt(abs(mat[0]**2)+abs(mat[1]**2)+abs(mat[2]**2))
            module_rcnn = math.sqrt(abs(mat[3]**2)+abs(mat[4]**2)+abs(mat[5]**2))
  
            # rospy.logdebug("------------------------")
            # rospy.logdebug("module_aru : %f", module_aru)
            # rospy.logdebug("module_rcnn : %f", module_rcnn)
            
            # rcnn = 0
            if mat[2] == 0:
                covNeural = 10
                covAruco = 0.01*abs(self.kalman.x[2])+1
                rospy.logdebug("*****Neural = 0 ou stoped!*****")

            # aruco = 0
            elif mat[5] == 0: # or abs(mat[2]-mat[5])>0.5:
                covNeural = (1.5/(abs(self.kalman.x[2])+1))+1
                covAruco = 10
                rospy.logdebug("*****aruco = 0 ou stoped!*****")

            else: #mat[2] != 0 and mat[5] != 0: 
                # greater neural error and lower aruco error at low height
                covNeural = (3.5/(abs(self.kalman.x[2])+0.1))+1.5 #np.exp(abs(self.kalman.x[2])*0.5-1)
                covAruco = 0.005*abs(self.kalman.x[2])+0.3
                rospy.logdebug("*****all-run!*****")


            # module_aru_ant = module_aru
            # module_rcnn_ant = module_rcnn

            # rospy.logdebug("------------------------")
            # rospy.logdebug("module_aru_ant : %f", module_aru_ant)
            # rospy.logdebug("module_rcnn_ant : %f", module_rcnn_ant)

            # set values of cov in R matrix
            arrayNeral = np.full((1, 3), covNeural, dtype=float)
            arrayAruco = np.full((1, 3), covAruco, dtype=float)
            Zarray = np.concatenate((arrayNeral, arrayAruco), axis=None)
            self.kalman.R = np.diag(Zarray)

            rospy.logdebug("arrayNeral : %f", covNeural)
            rospy.logdebug("arrayAruco : %f", covAruco)
            rospy.logdebug("------------------------")

            self.kalman.predict()
            self.kalman.update(mat)
            
            vec = Vector3()
            vec.x = self.kalman.x[0]
            vec.y = self.kalman.x[1]
            vec.z = self.kalman.x[2]

            # rospy.logdebug("------------------------")
            # rospy.logdebug("kalman.sensor[1].x : %f", vec.x)
            # rospy.logdebug("kalman.sensor[1].y : %f", vec.y)
            # rospy.logdebug("kalman.sensor[1].z : %f", vec.z)

            ##################################################################################

            hybrid_odom = Odometry()
            hybrid_odom.header.stamp = rospy.Time.now()
            hybrid_odom.header.frame_id = "hybrid_odom"
            hybrid_odom.header.seq = Keyframe
            hybrid_odom.child_frame_id = "odom"

            # set the position
            hybrid_odom.pose.pose = Pose(vec, self.OriAruco)

            tf_hybrid_to_drone.sendTransform(
                          (self.kalman.x[0],self.kalman.x[1],self.kalman.x[2]), 
                          (self.OriAruco.x,self.OriAruco.y,self.OriAruco.z,self.OriAruco.w), 
                          hybrid_odom.header.stamp, 
                          "hybrid_odom",
                          "odom") #world

            ##################################################################################

            rcnn_odom = Odometry()
            rcnn_odom.header.stamp = rospy.Time.now()
            rcnn_odom.header.frame_id = "rcnn_odom"
            rcnn_odom.header.seq = Keyframe
            rcnn_odom.child_frame_id = "odom"

            # set the position
            rcnn_odom.pose.pose = Pose(self.VecNeural, self.OriAruco)

            tf_rcnn_to_drone.sendTransform(
                          (self.VecNeural.x,self.VecNeural.y,self.VecNeural.z), 
                          (self.OriAruco.x,self.OriAruco.y,self.OriAruco.z,self.OriAruco.w), 
                          rcnn_odom.header.stamp, 
                          "rcnn_odom",
                          "odom") #world
            Keyframe += 1

            # publish the message
            self.odom_filter_pub.publish(hybrid_odom)
            self.odom_rcnn_pub.publish(rcnn_odom)
            self.pub_hibrid.publish(vec)

            r.sleep()

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def callbackPoseRCNN(self, data):
        # recive data
        objArray = Detection2DArray()

        # rcnn_pose
        objArray = data
        # rospy.logdebug(" lenth objArray.detections: %f", len(objArray.detections))
        
        if len(objArray.detections) != 0:
            # align coordinate axis X
            self.VecNeural.x = (-1)*(objArray.detections[0].results[0].pose.pose.position.x)
            self.VecNeural.y = objArray.detections[0].results[0].pose.pose.position.y
            VecNeuralz_current = objArray.detections[0].results[0].pose.pose.position.z
            # rospy.logdebug("--------------------------------")
            # rospy.logdebug("rcnn_pose.x (m): %f", VecNeuralx_current)
            # rospy.logdebug("rcnn_pose.y (m): %f", VecNeuraly_current)
            # rospy.logdebug("rcnn_pose.z (m): %f", VecNeuralz_current)

            # Filter list_z
            if len(self.list_z) < 20:
                self.list_z.append(VecNeuralz_current)
                # rospy.logdebug("--------------------------------")
                # rospy.logdebug('Size of list: %f',len(self.list_z))
                # rospy.logdebug('****Incomplete List')
                self.VecNeural_z_ant = VecNeuralz_current
            else:
                diff = abs(VecNeuralz_current-self.VecNeural_z_ant)
                rospy.logdebug('------------------------------')
                # rospy.logdebug('Diff points: %f',diff)

                if diff>1 and self.VecNeural_z_ant != 0:
                    self.VecNeural_z_ant = VecNeuralz_current
                    # self.VecNeural.z = 0
                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('****No capture!')

                else:
                    self.list_z.append(VecNeuralz_current)
                    del self.list_z[0]
                    VecNeuralz_current = sum(self.list_z)/len(self.list_z)
                    self.VecNeural.z = VecNeuralz_current

                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('Size of list_z:  %f',len(self.list_z))
                    # rospy.logdebug('Sum List list_z: %f',sum(self.list_z))
                    # rospy.logdebug('Med List list_z: %f',self.VecNeural.z)

                    self.VecNeural_z_ant = VecNeuralz_current

    def callbackPoseAruco(self, data):
        # recive data
        #aruco_pose = data

        # print "received data: ", data
        self.VecAruco = Vector3()
        self.VecAruco = Vector3(data.position.x, data.position.y, data.position.z)
        self.OriAruco = Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        # rospy.logdebug("--------------------------------")
        # rospy.logdebug("aruco_pose.x (m): %f", self.VecAruco.x)
        # rospy.logdebug("aruco_pose.y (m): %f", self.VecAruco.y)
        # rospy.logdebug("aruco_pose.z (m): %f", self.VecAruco.z)


if __name__ == '__main__':
    subscriber = Subscriber()
