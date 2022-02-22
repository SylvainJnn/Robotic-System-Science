#! /usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid


import math
import numpy as np


Xrobot, Yrobot, yaw = 0, 0, 0
obstacles_ranges = []
global_map = [[1],[-1]] #if local planner is choosed it needs to be initialises otherwise it does not compile

# ---------------- callback ----------------

def position_callback(info_pos):                # call back of the odom subscriber
    global Xrobot, Yrobot, roll, pitch, yaw  
    Xrobot = info_pos.pose.pose.position.x
    Yrobot = info_pos.pose.pose.position.y
    
    info_orientation = [info_pos.pose.pose.orientation.x,
                        info_pos.pose.pose.orientation.y,
                        info_pos.pose.pose.orientation.z,
                        info_pos.pose.pose.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(info_orientation) 


def Laser_callback(info_laser):                # call back of the laser subscriber
    global obstacles_ranges
    obstacles_ranges = info_laser.ranges

def my_map_callback(map_msg):                    #callback of the occupancygrid
    global global_map, width, height, resolution #create useful global variable
    global_map = np.reshape(map_msg.data, (map_msg.info.width, map_msg.info.height))             #reshape the map with the info within the flat map
    width, height, resolution = map_msg.info.width, map_msg.info.height, map_msg.info.resolution #get the infoof flat_map

# ---------------- define functions ----------------

def define_map():
    rospy.sleep(1)
    map = np.array(global_map)#create a local variable of the global map, this array is a numpy array
    # Create map on correct format
    map = map.astype(float)
    #convert map to correct format for planning 
    for i in range(len(global_map)):        
        for j in range(len(global_map[i])):
            if map[i, j] == 0:          #if the space is free put a 1
                map[i, j] = 1           # freespace
            else:                       #otherwise infinity
                map[i, j] = np.Infinity # obstacle
    return(map)


def init_var():
    goal = Point()                              #init goal
    goal.x = rospy.get_param("x_destination")   #take value from launch file
    goal.y = rospy.get_param("y_destination")
    speed = Twist()                             #init speed
    map = define_map()                          #init mat for global palnner

    return(goal, speed, map)

# ---------------- calculation functions ----------------

def calculate_position(r, theta, Xr, Yr, phi):
    #calculate the position of the object
    Xob = r * math.cos(theta) 
    Yob = r * math.sin(theta)
    
    pos_ob = np.array([Xob,Yob])    #position of the object in the robot frame
    
    pos_robot = np.array([Xr,Yr])   #position of the robot in the world frame
    R = np.array([[math.cos(phi),-1*(math.sin(phi))],
                    [math.sin(phi),math.cos(phi)]]) #transformation matrix
    pos_ob_origin = pos_robot + np.dot(R,pos_ob)    #position of the object in the word frame
    
    return(pos_ob_origin[0], pos_ob_origin[1])      #return the x and y position of the obstacles



def calculate_angle(point): #swap ? 
    x_dist = Xrobot - point.x   #do not use math.dist because we need negativ value #maybe I should use local variable and not the odom
    y_dist = Yrobot - point.y
    angle = np.arctan2(y_dist, x_dist) 
    return(angle)
 

def calculate_angle_destination(Fatt, Frep):    #calculate the angle from the yaw of the robot to the direction
    #sum the forces on X and Y
    F_x = Fatt[0] + Frep[0] 
    F_y = Fatt[1] + Frep[1]
    angle_destination = np.arctan2(F_y, F_x)    #compute the angle
    return(angle_destination)

def calculate_rotation(goal_angle, yaw_robot):          #calcul the directionof the rotation
    if(yaw_robot >= 0):                                 #if yaw is positive
        if((goal_angle < yaw_robot) and (goal_angle >= yaw_robot-np.pi)):
            rotation = -1
        else:
            rotation = +1
    else:
        if((goal_angle > yaw_robot) and (goal_angle <= yaw_robot+np.pi)):
            rotation = +1
        else:
            rotation = -1
    return(rotation)

def check_arrived(goal):                                #check if the robot has arrived
    epsilon = 0.5                                       #value use for the interval
    if(( Xrobot - epsilon <= goal.x <= Xrobot + epsilon ) and 
       ( Yrobot - epsilon <= goal.y <= Yrobot + epsilon )): #if closed enough
        speed.linear.x  = 0
        speed.angular.z = 0
        vel_pub.publish(speed)                               #set speed to 0
        print("OVER")
        return(False)
    return(True)




# ---------------- Forcefield  local ----------------

def attractive_Force(goal): #Attractive for computation 
    Katt = -10              #constant to tune  
    Fatt = Katt * (np.array([Xrobot, Yrobot]) - np.array([goal.x, goal.y])) #K(q - qgoal)
    return(Fatt[0], Fatt[1])  



def repulsive_Force(goal): 
    Q_ref = 2           #distance when the program start to pay attention
    eta = -0.05         #constant to tune
    
    # init variables
    obstacle = Point()   
    Frep = [0,0] 
    Frep_x = 0
    Frep_y = 0 
    angle = 0
    Frep_tpn = 0


    for i in range(len(obstacles_ranges)):      #for every case in the array
        theta = (i*(math.pi/719) - (math.pi/2)) #compute angle between the robot and the obstacles
        obstacle.x, obstacle.y = calculate_position(obstacles_ranges[i], theta, Xrobot, Yrobot, yaw)  #learn poiston of the obstacles
        qi_substraction = np.array([Xrobot, Yrobot]) - np.array([obstacle.x, obstacle.y]) #q - qi
        D_q = np.matmul(qi_substraction, qi_substraction.T)**(1/2)                        #D(q)

        if(D_q <= Q_ref):    # check if close enough 
            angle = calculate_angle(goal)   #calculate angle of the repulsive force for an obstacle
            Frep += eta * ((1/Q_ref - 1/D_q)*(qi_substraction/((D_q)**3)))  #calcul Frep

    return(Frep)


# ---------------- Forcefield global ----------------
#din each cell, add fx and Fy -> arctan for the direction and then la norme, et après le programme pour faire ça tant pis 
def att_map(real_pos_cell, goal):
    Katt = -10 #constant to tune  
    Fatt = Katt * (real_pos_cell - np.array([goal.x, goal.y]))
    return(Fatt[0], Fatt[1])  


def rep_map(real_pos_cell, obstacles_pos):
    Q_ref = 1
    eta = 1
    range_rep = 20
    qi_substraction = 0
    Frep = [0,0]
    for i in range(int(real_pos_cell[0] - range_rep), int(real_pos_cell[0] + range_rep)): #faire de pos -50, pos + 50 ?  for i in range(len(map) - value, len(map) + value)
        for j in range(int(real_pos_cell[1] - range_rep), int(real_pos_cell[1] + range_rep)):
            qi_substraction = np.array(obstacles_pos) - np.array([real_pos_cell[0], real_pos_cell[1]])
            D_q = np.matmul(qi_substraction, qi_substraction.T)**(1/2)
            if(D_q <= Q_ref):    # check if close enough #another way to do it is to take the avreage position of all the point is close enough and do one Frep = ... with only one angle
                angle = calculate_angle(goal)   #calculate angle of the repulsive force for an obstacle
                Frep = eta * ((1/Q_ref - 1/D_q)*(qi_substraction/((D_q)**3)))  #calcul Frep
                #Frep[0] = Frep[0] + Frep_tpn * math.cos(angle)                                # on x
                #Frep[1] = Frep[1] +Frep_tpn * math.sin(angle)                                 # on y 

    return(Frep)

def map_forcefield(map):
    map_gradients = define_map()

    for i in range(len(map)): 
        for j in range(len(map[i])):
            real_pos_cell = np.array([i* resolution - width/2, j* resolution - width/2 ])
            Fatt = att_map(real_pos_cell, goal) #add Fatt
            if(map[i, j] != 1):#if this point is an bostacle 
                obstacles_pos = [i, j]
                Frep = rep_map(real_pos_cell, obstacles_pos)    #add Frep
                angle = calculate_angle_destination(Fatt, [0,0])
            else:
                angle = calculate_angle_destination(Fatt, [0,0])
            map_gradients[i, j] = angle
    return(map_gradients)

                
                





# ---------------- mouvement local ----------------    

def robot_rotation(goal_angle, rotation):#first way point and reach witht he two functions // try to do poitn and roch at the same time by creating a good function for how to reach a point 
    print("robot_rotation", abs(goal_angle - yaw))
    while (abs(goal_angle - yaw) > 0.2):    #while yaw is no precises enough you turn
        speed.linear.x = 0.08               #there is always a linear speed otherwise the robot is stuck (change direction all the time)
        speed.angular.z =rotation* 0.15
        vel_pub.publish(speed)              #publish speed
    print('end rotation')

def robot_translation():
    print('start translation')
    old_point = Point()

    old_point.x = Xrobot
    old_point.y = Yrobot

    dist_robot = math.dist([Xrobot, Yrobot], [old_point.x, old_point.y])
    dist_todo = 0.1
    while(dist_robot < dist_todo):
        speed.linear.x = 0.2
        speed.angular.z = 0
        vel_pub.publish(speed)
        dist_robot = math.dist([Xrobot, Yrobot], [old_point.x, old_point.y])
    print('end translation')
        



def robot_control(goal):
    #call forces
    Fatt = np.array(attractive_Force(goal))
    Frep = np.array(repulsive_Force(goal))

    print("Attractives Forces :", Fatt)
    print("Repulsives Forces  :", Frep)
    goal_angle = calculate_angle_destination(Fatt, Frep)    #compute the goal angke
    print("goal_angle :", goal_angle)
    rotation = calculate_rotation(goal_angle, yaw)          #which direction it should turn
    
    robot_rotation(goal_angle, rotation)                    #rotation
    robot_translation()                           #translation

    
# ---------------- mouvement global ----------------   


def rotation_global(goal, yaw_robot): #rotation function
    rotation = calculate_rotation(goal, yaw_robot) #chech in which direction it should turn

    epsilon = math.pi/(2*10)                       #take 10% of PI/2
    print(goal, yaw_robot)
    
    print("start rotation")
    while not(((goal - epsilon) < yaw) and 
                yaw < (goal + epsilon)):    #while yaw is not close enough of the goal
           
        speed.linear.x = 0                             #stop the linear speed
        speed.angular.z = rotation * 0.09              #set the rotation speed
        vel_pub.publish(speed)                         #publish the spee

    print("end rotation")
    speed.linear.x = 0          
    speed.angular.z = 0
    vel_pub.publish(speed)      #reset the speed
    
def translation_global(Xmap, Ymap):
    print("start translation")
    speed.linear.x = 0.1                                #change spees    
    speed.angular.z = 0
    Xmap_new = int(np.round(Xrobot/resolution + width/2))       #create information of the actual cell
    Ymap_new = int(np.round(Yrobot/resolution + height/2 ))
    while(Xmap == Xmap_new or Ymap == Ymap_new):                #while the robot is on the same cell
        vel_pub.publish(speed)         #publish speed           #publishe the speed
        Xmap_new = int(np.round(Xrobot/resolution + width/2))   #update actual cell position
        Ymap_new = int(np.round(Yrobot/resolution + height/2 ))
        #print(Xmap_new)
    #rospy.sleep()




rospy.init_node('Assignement1_task3_node')  

odom_sub = rospy.Subscriber("/odom", Odometry, position_callback)
scan_sub = rospy.Subscriber("/scan", LaserScan, Laser_callback)
#my_map_sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map_callback)
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
my_map_sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map_callback)#Occupancygrid subscriber


goal, speed, map = init_var() #init main variables
rate = rospy.Rate(1)
moving = True   


global_planner = True

rate.sleep() 

if(global_planner):
    while(moving):                 # while it is moving 

        rate.sleep()               
        robot_control(goal)
        #print(Xrobot, Yrobot)
        moving = check_arrived(goal)    #checked if the robot is arrived, if yes : moving <- False
        print("moving :", moving)

else:
    map_gradients = map_forcefield(map)

    goal.x = goal.x/resolution + width/2
    goal.y = goal.y/resolution + width/2

    while(moving):
        rate.sleep() 

        Xmap = int(np.round(Xrobot/resolution + width/2))
        Ymap = int(np.round(Yrobot/resolution + height/2 ))
        Xmap_old = Xmap
        Ymap_old = Ymap
        
        goal_angle = map_gradients[Xmap, Ymap]
        print(goal_angle)
        rotation_global(goal_angle, yaw)

        translation_global(Xmap, Ymap)


        
        moving = check_arrived(goal)

