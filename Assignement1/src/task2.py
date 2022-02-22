#! /usr/bin/env python

import rospy  
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import OccupancyGrid 


from tf.transformations import euler_from_quaternion, quaternion_from_euler

global length, resolution   #global variable used for :
length = 300                #the size of the map
resolution = 0.05           #the resolution of the map

def get_position(info_pos):                  # call back of the odom subscriber
    global Xrobot, Yrobot, roll, pitch, yaw  #put the useful variable as global in order to reuse them
    Xrobot = info_pos.pose.pose.position.x
    Yrobot = info_pos.pose.pose.position.y
    
    info_orientation = [info_pos.pose.pose.orientation.x,
                        info_pos.pose.pose.orientation.y,
                        info_pos.pose.pose.orientation.z,
                        info_pos.pose.pose.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(info_orientation) #transform quaternion into euler angles 


def Laser_callback(info_laser):             # call back of the laser subscriber
    global obstacles_ranges                 
    obstacles_ranges = info_laser.ranges    #put the info_laser.ranges into the global variables


def update_map(r, theta, Xr, Yr, phi):
    # calculate the position of the object
    Xob = r * math.cos(theta)      
    Yob = r * math.sin(theta)
    
    pos_ob = np.array([Xob,Yob])    #position of the object in the robot frame
    
    pos_robot = np.array([Xr,Yr])   #position of the robot in the world frame
    R = np.array([[math.cos(phi),-1*(math.sin(phi))],   
                    [math.sin(phi),math.cos(phi)]])             #transformation matrix
    pos_ob_origin = pos_robot + np.dot(R,pos_ob)                #position of the object in the word frame
    
    xow = int((pos_ob_origin[0]/resolution + (map.shape[0])/2)) #transform the x position to put it in the map
    yow = int((pos_ob_origin[1]/resolution + (map.shape[1]/2))) #transform the y position to put it in the map
    
    if(xow < np.size(map,0) and yow < np.size(map,1)):  #if in the map range, add it to the map
        map[xow,yow] = 125                              #the value is 125 because to pass the map to task 4 we used occupancy grid --> we need the value to be coded on 8 bits


def define_map(length):  #define map 
    
    s = (length,length)
    map = np.zeros(s,dtype=np.uint8)
    map = np.array(map)
    for i in range(len(map)):               #put 0 everywhere
        for j in range(len(map[i])):
            map[i,j] = 0
    return(map)


rospy.init_node('Assignement1_task2_node')  


odom_sub = rospy.Subscriber('/odom', Odometry, get_position, queue_size=1)          #odom subscriber 
laser_sub = rospy.Subscriber('/scan', LaserScan, Laser_callback, queue_size=1)      #laser subscriber 
map_pub = rospy.Publisher("/my_map", OccupancyGrid ,queue_size = 1 )                #occupancy grid publisher

#define map and flat_map used in task4
global map
map = define_map(length)

flat_map = OccupancyGrid()  #used in task5 // create an OccupancyGrid object 
#set the info resolution, width and height in the occupancy grid object 
flat_map.info.resolution = resolution       
flat_map.info.width = length
flat_map.info.height = length 

        

        

rate = rospy.Rate(2)

while(not rospy.is_shutdown()):    
    rate.sleep()        
    Xr = Xrobot                 #use local varables to compute the position of obstacles
    Yr = Yrobot
    phi = yaw  
    obstacles = obstacles_ranges 

    for i in range(len(obstacles)):         #for all the laser  
       if not math.isinf(obstacles[i]):     #for every object detected 
            theta = (i*(math.pi/719) - (math.pi/2)) #compute theta
            update_map(obstacles[i], theta, Xr, Yr, phi)  #call the update_map function
            
    img = Image.fromarray(np.uint8( map ))                #create and save the map
    img.save('/home/user/catkin_ws/src/assignment_student/pictures/grid.png')
    print("update task2")

    
    flat_map.data = map.flatten() #update the the flatmap 
    map_pub.publish(flat_map)     #publish the flat map
    print("published")



