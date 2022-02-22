#! /usr/bin/env python

import rospy  
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_position(info_pos):                 # call back of the odom subscriber
    global Xrobot, Yrobot, roll, pitch, yaw #put the useful variable as global in order to reuse them
    Xrobot = info_pos.pose.pose.position.x
    Yrobot = info_pos.pose.pose.position.y
    
    info_orientation = [info_pos.pose.pose.orientation.x,
                        info_pos.pose.pose.orientation.y,
                        info_pos.pose.pose.orientation.z,
                        info_pos.pose.pose.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(info_orientation) #transform quaternion into euler angles 

rospy.init_node('Assignement1_task1_node')  
odom_sub = rospy.Subscriber('/odom', Odometry, get_position, queue_size=1)  #odom subscriber
rate = rospy.Rate(1)

while(not rospy.is_shutdown()):    
    rate.sleep()                # sleep of one second 
    print("Xrobot :", Xrobot)     #print the X, Y and yaw of the robot
    print("Yrobot :", Yrobot)
    print("yaw :", yaw) 