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

        self.PARENT_NAME = rospy.get_param('~parent_name', "odom")

        self.kalman = Kalman(n_states = 3, n_sensors = 6)
        self.kalman.P *= 10
        self.kalman.R *= 0.02

        self.VecNeural = Vector3()
        self.VecAruco = Vector3()
        self.OriAruco = Quaternion(0,0,0,1)

        self.Keyframe_aruco = 0
        self.Keyframe_rcnn = 0

        self.aruco_odom = Odometry()
        self.aruco_odom.header.stamp = rospy.Time.now()
        self.aruco_odom.header.frame_id = "aruco_odom"
        self.aruco_odom.header.seq = self.Keyframe_aruco
        self.aruco_odom.child_frame_id = self.PARENT_NAME

        self.rcnn_odom = Odometry()
        self.rcnn_odom.header.stamp = rospy.Time.now()
        self.rcnn_odom.header.frame_id = "rcnn_odom"
        self.rcnn_odom.header.seq = self.Keyframe_rcnn
        self.rcnn_odom.child_frame_id = self.PARENT_NAME

        self.list_x = []
        self.list_y = []
        self.list_z = []

        self.list_time_aruco = []

        self.VecNeural_x_previous = 0
        self.VecNeural_y_previous = 0
        self.VecNeural_z_previous = 0

        # Publishers
        self.pub_hibrid = rospy.Publisher('kalman/hybrid', Vector3)
        self.odom_filter_pub = rospy.Publisher("odom_filter", Odometry)
        self.odom_rcnn_pub = rospy.Publisher("odom_rcnn", Odometry)
        self.odom_aruco_pub = rospy.Publisher("odom_aruco", Odometry)
        self.p_aruco = rospy.Publisher("time/aruco", Vector3)

        # transform tf
        tf_hybrid_to_drone = tf.TransformBroadcaster()
        
        Keyframe = 0
        
        rospy.Subscriber("rcnn/objects", Detection2DArray, self.callbackPoseRCNN)
        rospy.Subscriber("aruco_double/pose",Pose, self.callbackPoseAruco)
        
        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            neural = [self.VecNeural.x, self.VecNeural.y, self.VecNeural.z]
            aruco = [self.VecAruco.x, self.VecAruco.y, self.VecAruco.z]

            mat = np.matrix(np.concatenate((neural, aruco), axis=None)).getT()

            if self.kalman.first:
                # insert the first 3 values of the vector 
                self.kalman.x = mat[0:3, :]
                self.kalman.first = False

            current_time = rospy.Time.now()
            dt_aruco = (current_time-self.aruco_odom.header.stamp).to_sec()
            dt_rcnn = (current_time-self.rcnn_odom.header.stamp).to_sec()

            if len(self.list_time_aruco) < 6:
                self.list_time_aruco.append(dt_aruco)
            else:
                self.list_time_aruco.append(dt_aruco)
                del self.list_time_aruco[0]
                med_time = sum(self.list_x)/len(self.list_x)

            rospy.logdebug("------------------------")
            rospy.logdebug("time of aru : %f", dt_aruco)
            rospy.logdebug("frequencia :  %f", (med_time))

            # module_rcnn = math.sqrt(abs(mat[3]**2)+abs(mat[4]**2)+abs(mat[5]**2))
            # module_aru = math.sqrt(abs(mat[0]**2)+abs(mat[1]**2)+abs(mat[2]**2))
  
            # rospy.logdebug("------------------------")
            # rospy.logdebug("module_aru : %f", module_aru)
            # rospy.logdebug("module_rcnn : %f", module_rcnn)
            
            ##################################################################################

            # rcnn = 0
            if mat[2] == 0 or dt_rcnn > 5:
                covNeural = 10
                covAruco = 0.01*abs(self.kalman.x[2])+1
                # rospy.logdebug("*****Neural = 0 ou stoped!*****")

            # aruco = 0
            elif mat[5] == 0 or dt_aruco > 5:
                covNeural = (1.5/(abs(self.kalman.x[2])+1))+1
                covAruco = 10
                # rospy.logdebug("*****aruco = 0 ou stoped!*****")

            else:
                # greater neural error and lower aruco error at low height
                covNeural = (3.5/(abs(self.kalman.x[2])+0.1))+1.5 #np.exp(abs(self.kalman.x[2])*0.5-1)
                covAruco = 0.005*abs(self.kalman.x[2])+0.3
                # rospy.logdebug("*****all-run!*****")

            ##################################################################################

            # module_aru_ant = module_aru
            # module_rcnn_ant = module_rcnn

            # set values of cov in R matrix
            arrayNeral = np.full((1, 3), covNeural, dtype=float)
            arrayAruco = np.full((1, 3), covAruco, dtype=float)
            Zarray = np.concatenate((arrayNeral, arrayAruco), axis=None)
            self.kalman.R = np.diag(Zarray)

            # rospy.logdebug("arrayNeral : %f", covNeural)
            # rospy.logdebug("arrayAruco : %f", covAruco)
            # rospy.logdebug("------------------------")

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
            if dt_aruco < 0.1 or dt_rcnn < 0.1:
                hybrid_odom = Odometry()
                hybrid_odom.header.stamp = rospy.Time.now()
                hybrid_odom.header.frame_id = "hybrid_odom"
                hybrid_odom.header.seq = Keyframe
                hybrid_odom.child_frame_id = self.PARENT_NAME

                explicit_quat = [self.OriAruco.x, self.OriAruco.y, self.OriAruco.z, self.OriAruco.w]
                euler = tf.transformations.euler_from_quaternion(explicit_quat)
                # roll = euler[0]
                # pitch = euler[1]
                yaw = euler[2]

                # # since all odometry is 6DOF we'll need a quaternion created from yaw
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, -yaw)

                # set the position
                hybrid_odom.pose.pose = Pose(vec, Quaternion(*odom_quat))

                tf_hybrid_to_drone.sendTransform(
                              (self.kalman.x[0],self.kalman.x[1],self.kalman.x[2]), 
                              odom_quat, 
                              hybrid_odom.header.stamp, 
                              "hybrid_odom",
                              self.PARENT_NAME) #world

                ##################################################################################

                Keyframe += 1

                # publish the message
                self.odom_filter_pub.publish(hybrid_odom)
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
        size_filter = 10
        
        if len(objArray.detections) != 0:
            # align coordinate axis X
            neuralx_current = (-1)*(objArray.detections[0].results[0].pose.pose.position.x)
            neuraly_current = objArray.detections[0].results[0].pose.pose.position.y
            neuralz_current = objArray.detections[0].results[0].pose.pose.position.z
            # rospy.logdebug("--------------------------------")
            # rospy.logdebug("rcnn_pose.x (m): %f", VecNeuralx_current)
            # rospy.logdebug("rcnn_pose.y (m): %f", VecNeuraly_current)
            # rospy.logdebug("rcnn_pose.z (m): %f", neuralz_current)

            ##############################################################

            # Filter list_x
            if len(self.list_x) < size_filter:
                self.list_x.append(neuralx_current)
                # rospy.logdebug("--------------------------------")
                # rospy.logdebug('Size of list: %f',len(self.list_x))
                # rospy.logdebug('****Incomplete List')
                self.VecNeural_x_previous = neuralx_current
            else:
                diff = abs(neuralx_current-self.VecNeural_x_previous)
                # rospy.logdebug('------------------------------')
                # rospy.logdebug('Diff points: %f',diff)

                if diff>0.2 and self.VecNeural_x_previous != 0:
                    self.VecNeural_x_previous = neuralx_current
                    # self.VecNeural.x = 0
                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('****No capture!')

                else:
                    self.list_x.append(neuralx_current)
                    del self.list_x[0]
                    neuralx_current = sum(self.list_x)/len(self.list_x)
                    self.VecNeural.x = neuralx_current

                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('Size of list_x:  %f',len(self.list_x))
                    # rospy.logdebug('Sum List list_x: %f',sum(self.list_x))
                    # rospy.logdebug('Med List list_x: %f',self.VecNeural.x)

                    self.VecNeural_x_previous = neuralx_current

            ##############################################################

            # Filter list_y
            if len(self.list_y) < size_filter:
                self.list_y.append(neuraly_current)
                # rospy.logdebug("--------------------------------")
                # rospy.logdebug('Size of list: %f',len(self.list_y))
                # rospy.logdebug('****Incomplete List')
                self.VecNeural_y_previous = neuraly_current
            else:
                diff = abs(neuraly_current-self.VecNeural_y_previous)
                # rospy.logdebug('------------------------------')
                # rospy.logdebug('Diff points: %f',diff)

                if diff>0.2 and self.VecNeural_y_previous != 0:
                    self.VecNeural_y_previous = neuraly_current
                    # self.VecNeural.y = 0
                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('****No capture!')

                else:
                    self.list_y.append(neuraly_current)
                    del self.list_y[0]
                    neuraly_current = sum(self.list_y)/len(self.list_y)
                    self.VecNeural.y = neuraly_current

                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('Size of list_y:  %f',len(self.list_y))
                    # rospy.logdebug('Sum List list_y: %f',sum(self.list_y))
                    # rospy.logdebug('Med List list_y: %f',self.VecNeural.y)

                    self.VecNeural_y_previous = neuraly_current

            ##############################################################

            # Filter list_z
            if len(self.list_z) < size_filter:
                self.list_z.append(neuralz_current)
                # rospy.logdebug("--------------------------------")
                # rospy.logdebug('Size of list: %f',len(self.list_z))
                # rospy.logdebug('****Incomplete List')
                self.VecNeural_z_previous = neuralz_current
            else:
                diff = abs(neuralz_current-self.VecNeural_z_previous)
                # rospy.logdebug('------------------------------')
                # rospy.logdebug('Diff points: %f',diff)

                if diff>0.4 and self.VecNeural_z_previous != 0:
                    self.VecNeural_z_previous = neuralz_current
                    # self.VecNeural.z = 0
                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('****No capture!')

                else:
                    self.list_z.append(neuralz_current)
                    del self.list_z[0]
                    neuralz_current = sum(self.list_z)/len(self.list_z)
                    self.VecNeural.z = neuralz_current

                    # rospy.logdebug('------------------------------')
                    # rospy.logdebug('Size of list_z:  %f',len(self.list_z))
                    # rospy.logdebug('Sum List list_z: %f',sum(self.list_z))
                    # rospy.logdebug('Med List list_z: %f',self.VecNeural.z)

                    self.VecNeural_z_previous = neuralz_current

            self.rcnn_odom.header.stamp = rospy.Time.now()
            self.rcnn_odom.header.seq = self.Keyframe_rcnn

            # stabilize angles and align transformations
            explicit_quat = [self.OriAruco.x, self.OriAruco.y, self.OriAruco.z, self.OriAruco.w]
            euler = tf.transformations.euler_from_quaternion(explicit_quat)
            # roll = euler[0]
            # pitch = euler[1]
            yaw = euler[2]

            # # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, -yaw)

            # set the position
            self.rcnn_odom.pose.pose = Pose(self.VecNeural, Quaternion(*odom_quat))

            tf_rcnn_to_drone = tf.TransformBroadcaster()
            tf_rcnn_to_drone.sendTransform(
                          (self.VecNeural.x,self.VecNeural.y,self.VecNeural.z), 
                          odom_quat, 
                          self.rcnn_odom.header.stamp, 
                          "rcnn_odom",
                          self.PARENT_NAME) #world

            self.Keyframe_rcnn+=1    

            self.odom_rcnn_pub.publish(self.rcnn_odom)

    def callbackPoseAruco(self, data):
        # recive data
        #aruco_pose = data
        init_time = rospy.Time.now()
        # print "received data: ", data
        self.VecAruco = Vector3(data.position.x, data.position.y, data.position.z)
        self.OriAruco = Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        # rospy.logdebug("--------------------------------")
        # rospy.logdebug("aruco_pose.x (m): %f", self.VecAruco.x)
        # rospy.logdebug("aruco_pose.y (m): %f", self.VecAruco.y)
        # rospy.logdebug("aruco_pose.z (m): %f", self.VecAruco.z)

        self.aruco_odom.header.stamp = rospy.Time.now()
        self.aruco_odom.header.seq = self.Keyframe_aruco

        # stabilize angles and align transformations
        explicit_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quat)
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, -yaw)

        # set the position
        self.aruco_odom.pose.pose = Pose(self.VecAruco, Quaternion(*odom_quat))

        tf_aruco_to_drone = tf.TransformBroadcaster()
        tf_aruco_to_drone.sendTransform(
                      (data.position.x, data.position.y, data.position.z), 
                      odom_quat, 
                      self.aruco_odom.header.stamp, 
                      "aruco_odom",
                      self.PARENT_NAME) #world

        self.Keyframe_aruco+=1
        self.odom_aruco_pub.publish(self.aruco_odom)
        
        pAruco = Vector3()
        current_time = rospy.Time.now()
        dt_aruco = (current_time-init_time).to_sec()
        #f=1/T
        pAruco.x = dt_aruco   
        pAruco.y = (1/dt_aruco) 
        pAruco.y = data.position.z
        self.p_aruco.publish(pAruco)

if __name__ == '__main__':
    subscriber = Subscriber()
