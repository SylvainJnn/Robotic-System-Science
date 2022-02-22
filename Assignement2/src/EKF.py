#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from geonav_transform__geonav_conversions import *

import csv

class EKF:
    def __init__(self):
        rospy.init_node('Assignement2_task1_node') 

        #Publishers
        self.pub_odom_SJ = rospy.Publisher("/odometry_Sylvain_Jannin", Odometry, queue_size = 1)
        self.pub_gps_SJ = rospy.Publisher("/gps_Sylvain_Jannin", Odometry, queue_size = 1)
        self.pub_fusion_SJ = rospy.Publisher("/fused_pose_Sylvain_Jannin", Odometry, queue_size = 1) 
        #self.pub_NavSat = rospy.Publisher("/NavSatFix_Sylvain_Jannin", NavSatFix, queue_size = 1) 

        #Subscribers
        self.sub_visual_odom = rospy.Subscriber("/visual_odometry", Odometry, self.get_odometry_Sylvain_Jannin)
        self.sub_get_gps = rospy.Subscriber("/gps", NavSatFix, self.get_gps_Sylvain_Jannin)

        

        #variables
        #odom/motion variables
        self.twist_covariance = [[0,0,0],
                                 [0,0,0],
                                 [0,0,0]] 
        self.odom_predicted = Odometry()
        self.odom_init = True       #after the first callback the variable will be False
        
        self.jacobian_odom =[[0,0,0],
                             [0,0,0],
                             [0,0,0]]
        self.yaw_odom = 0
        
        #gps variables
        self.pose_covariance = [[1,0],
                                [0,1]]   
        self.gps_pose = Point()     #position from the GPD
        self.gps_mesure = Odometry()
        self.gps_init = True        #after the first callback the variable will be False
        self.latitude_origin = 0    #latitude and longitude at the begning, initialise during the false callback
        self.longitude_origin = 0

        self.jacobian_gps = [[0,0,0], 
                             [0,0,0]]

        #EKF variables
        self.mu_corrected = [0,0,0]         
        self.covariance_corrected = np.array([[700,0,0],
                                              [0,700,0],
                                              [0,0,700]])
        mu = [0, 0, 0]

        #fusion publisher variables
        self.Fusion_EKF = Odometry()
        #self.pub_NavSat = NavSatFix()
        
        #variable for CSV file
        self.obj = open('EKF_csv.csv', 'w')
        self.my_csv = csv.writer(self.obj)
        self.my_csv.writerow( ["timestamp", "x", "y", "yaw"])   #write the first line containing the name of the infirmation   


    def get_odometry_Sylvain_Jannin(self, odom_msg):
        #covariance -> constant to thune by ourself
        covariance = [[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]]        
        coef = 1
        self.twist_covariance = np.array(covariance)*coef

        #compute odom of next step
        #take twist message of visual_odometry since we don't change it 
        self.odom_predicted.twist = odom_msg.twist

        #the goal is to guess the next step on (x,y,yaw), for this we need to compute all the orientation
        odom_orientation = [self.odom_predicted.pose.pose.orientation.x,
                            self.odom_predicted.pose.pose.orientation.y,
                            self.odom_predicted.pose.pose.orientation.z,
                            self.odom_predicted.pose.pose.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(odom_orientation) #transform quaternion into euler angles 

        if(self.odom_init):
            #yaw = np.pi/6   #add offset to fit the GPS #not usefull
            self.odom_init = False

        #calculate next position thanks to linear speed
        #pytagore theorem but with numpy  library step
        linear_speed_x = np.power(odom_msg.twist.twist.linear.x, 2)                    #Vx**2             
        linear_speed_y = np.power(odom_msg.twist.twist.linear.y, 2)                    #Vy**2  
        linear_speed = np.add(linear_speed_x, linear_speed_y)                          #Vx + Vy
        linear_speed = np.sqrt(linear_speed)                                           #sqrt(Vx + Vy)
        self.odom_predicted.pose.pose.position.x += np.cos(yaw) * linear_speed         #compute speed on x axis
        self.odom_predicted.pose.pose.position.y += np.sin(yaw) * linear_speed         #compute speed on y axis

        #add orientation information
        roll -= odom_msg.twist.twist.angular.x 
        pitch -= odom_msg.twist.twist.angular.y 
        yaw -= odom_msg.twist.twist.angular.z 
        self.yaw_odom = yaw

        (qx, qy, qz, qw) = quaternion_from_euler(roll, pitch, yaw)  #transform euler into quaternions angles 
        #regive the updateted quaternions angles
        self.odom_predicted.pose.pose.orientation.x = qx
        self.odom_predicted.pose.pose.orientation.y = qy
        self.odom_predicted.pose.pose.orientation.z = qz
        self.odom_predicted.pose.pose.orientation.w = qw

        #publlish new odom on odometry_Sylvain_Jannin 
        #add header in "odom" information in order to visualize it in RVIZ
        self.odom_predicted.header = odom_msg.header
        self.odom_predicted.header.frame_id = "odom"
        self.odom_predicted.child_frame_id = "odom"
        self.pub_odom_SJ.publish(self.odom_predicted)

        #Jacobian
        c_yaw = np.cos(yaw)
        s_yaw = np.sin(yaw)

        dx = linear_speed
        dy = 0 #for me it is 0 but in matlab we have something so weither dx = self.odom_predicted.pose.pose.position.x or dx = linear speed

        a = -s_yaw*dx - c_yaw*dy #-sin(yaw)*dx -cos(yaw)*dy  
        b = c_yaw*dx - s_yaw*dy #cos(yaw)*dx - sin(yaw)*dy 

        G = [[1,0,a],       
             [0,1,b],    
             [0,0,1]] 
        self.jacobian_odom = G


        self.EKF_Filter(linear_speed, odom_msg.twist.twist.angular.z) #call my EKF filter



    def get_gps_Sylvain_Jannin(self, gps_msg):
        #covariance -> constant to thune by ourself
        covariance = [[1, 0],
                      [0, 1]]      
        coef = 1000
        self.pose_covariance = np.array(covariance)*coef 

        #take actual latitude and longitude
        latitude_actual = gps_msg.latitude
        longitude_actual = gps_msg.longitude

        #if it is the first time, we keep origin latitude and longitude
        if(self.gps_init):
            self.latitude_origin = latitude_actual
            self.longitude_origin = longitude_actual
            self.gps_init = False

        #compute actual x and y position
        self.gps_pose.x, self.gps_pose.y = ll2xy(latitude_actual, longitude_actual, self.latitude_origin, self.longitude_origin)
        
        self.gps_mesure.pose.pose.position.x = self.gps_pose.x
        self.gps_mesure.pose.pose.position.y = self.gps_pose.y
        
        #add header and information to publish it on RVIZ
        self.gps_mesure.header = gps_msg.header
        self.gps_mesure.header.frame_id = "odom"
        self.gps_mesure.child_frame_id = "odom"

        self.pub_gps_SJ.publish(self.gps_mesure)

        H = [[1, 0, 0],
             [0, 1, 0]]      
        self.jacobian_gps = np.array(H)
        

    def publish_EKF(self):
        #take mu in local variable
        fusion_x = self.mu_corrected[0]
        fusion_y = self.mu_corrected[1]     
        fusion_yaw = self.mu_corrected[2] 
        (qx, qy, qz, qw) = quaternion_from_euler(0, 0, fusion_yaw) #transforme euler to quaternion

        #set the variable in the odometry variables in oder to be published
        self.Fusion_EKF.pose.pose.position.x = fusion_x
        self.Fusion_EKF.pose.pose.position.y = fusion_y

        self.Fusion_EKF.pose.pose.orientation.x = qx
        self.Fusion_EKF.pose.pose.orientation.y = qy
        self.Fusion_EKF.pose.pose.orientation.z = qz
        self.Fusion_EKF.pose.pose.orientation.w = qw
        
        #add header and information to publish it on RVIZ
        self.Fusion_EKF.header = self.odom_predicted.header
        self.pub_fusion_SJ.publish(self.Fusion_EKF) 

        #write in CSV file
        timestamp = str(self.Fusion_EKF.header.stamp.secs) + str(self.Fusion_EKF.header.stamp.nsecs)    #add time in secs and in nanosecs, to add them I add them in string 
        my_information = [timestamp, fusion_x, fusion_y, fusion_yaw]
        self.my_csv.writerow(my_information)


    def EKF_prediction(self, linear_speed, angular_speed):#motion model
        #compute mu_predicted from dead recogning
        mu_corrected = self.mu_corrected    #take the corrected values in local variable        
        yaw = mu_corrected[2]
        
        mu_x = mu_corrected[0] + np.cos(yaw) * linear_speed                              #compute speed on x axis
        mu_y = mu_corrected[1] + np.sin(yaw) * linear_speed                              #compute speed on y axis
        mu_yaw = mu_corrected[2] - angular_speed
        
        mu = [mu_x, mu_y, mu_yaw]       
        self.mu_predicted = np.array(mu)
        
        #odometry Jacobian
        G = np.array(self.jacobian_odom)


        #camculate R    
        R = self.twist_covariance

        step_cov = np.matmul(G, self.covariance_corrected)
        step_cov = np.matmul(step_cov, G.T)
        self.covariance_predicted = np.add(step_cov, R) 

        #call the publisher in prediction to have a smoother line
        self.publish_EKF() 
    
    def EKF_correction(self):     
        #compute jaconbian, 
        H = np.array(self.jacobian_gps)

        #compute Q
        Q = np.array(self.pose_covariance)


        #compute K
        K_step1 = np.matmul(self.covariance_predicted, H.T)
        k_step2 = np.matmul(H, self.covariance_predicted) 
        k_step21 = np.matmul(k_step2, H.T)
        Innovation_S = np.add(k_step21, Q) 
        k_step3 = np.linalg.inv(Innovation_S)           #do the inverse of the matrix
        K = np.matmul(K_step1, k_step3)
        
        #compute mu
        #compute Innovation_v -> first compute z and h_mu_predicted
        z_x = self.gps_mesure.pose.pose.position.x          #from gps model
        z_y = self.gps_mesure.pose.pose.position.y
        z = np.array([z_x, z_y])  

        h_mu_predicted = np.matmul(H, self.mu_predicted)    # H x mu_predicted // from motion model

        Innovation_v = np.subtract(z, h_mu_predicted)
        mu_cor_step = np.matmul(K, Innovation_v)
        self.mu_corrected = np.add(self.mu_corrected, mu_cor_step)
        
        #compute covariance
        cov_corr_step = np.matmul(K, H)

        I = np.eye(3) 
        cov_corr_step = np.subtract(I, cov_corr_step)        
        self.covariance_corrected = np.matmul(cov_corr_step, self.covariance_predicted)


    def EKF_Filter(self, linear_speed, angular_speed):
        self.EKF_prediction(linear_speed, angular_speed)
        self.EKF_correction()
    

        

if (__name__ == '__main__'):
    my_EKF_filter = EKF()
    print("EKF is running")
    rospy.spin()


