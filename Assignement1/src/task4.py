#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from random import random
import math
from PathPlanning import PathPlanning

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist   
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from nav_msgs.msg import OccupancyGrid #to import map from task2.py

from PIL import Image, ImageDraw
# Control parameters for the PID controller
# These parameters will need to be tuned on real robot
Kv = 9
Ka = 5
Kb = -0.5
dt = 0.1
global_map = 0
show_animation = True

# --------------- callback ---------------

def get_position(info_pos):                     # call back of the odom subscriber
    global Xrobot, Yrobot, roll, pitch, yaw     #put the useful variable as global in order to reuse them
    Xrobot = info_pos.pose.pose.position.x
    Yrobot = info_pos.pose.pose.position.y
    
    info_orientation = [info_pos.pose.pose.orientation.x,
                        info_pos.pose.pose.orientation.y,
                        info_pos.pose.pose.orientation.z,
                        info_pos.pose.pose.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(info_orientation) #transform quaternion into euler angles 
    
def my_map_callback(map_msg):                    #callback of the occupancygrid
    global global_map, width, height, resolution #create useful global variable
    global_map = np.reshape(map_msg.data, (map_msg.info.width, map_msg.info.height))             #reshape the map with the info within the flat map
    width, height, resolution = map_msg.info.width, map_msg.info.height, map_msg.info.resolution #get the infoof flat_map


# --------------- point and shoot ---------------

def translation(goal_pose, which):
    #if it is a trasnlation folowing the x axis, the program is gonna look at the y axis only, otherwise it wil check the x axis only
    print("Start translation in ", which)   
    print(goal_pose)
    speed.linear.x = 0.09      
    speed.angular.z = 0
    vel_pub.publish(speed)         #publish speed
    moving = True
    while(moving):
        vel_pub.publish(speed)  # I had issues with the publisher : sometimes it was not publying, so i piblish all the time in case
        #set the x and y position to be fit to the map
        X = Xrobot/resolution + width/2
        Y = Yrobot/resolution + width/2 # height /2 
        if(which == "y"):
            if(np.round(Y) == goal_pose[1]): #if the Y is close enough the the goal position 
                moving = False #the robot reached the step           
        else:
            if(np.round(X) == goal_pose[0]): #if the X is close enough the the goal position 
                moving = False


    print("translation did")
    speed.linear.x = 0 #reset
    speed.angular.z = 0
    vel_pub.publish(speed)
    return(Xrobot, Yrobot)


def calculate_rotation(goal_angle, yaw_robot):      #calcul the directionof the rotation
    if(yaw_robot >= 0):                             #if yaw is positive
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

    
def rotation(goal, yaw_robot): #rotation function
    rotation = calculate_rotation(goal, yaw_robot) #chech in which direction it should turn
    
    speed.linear.x = 0                             #stop the linear speed
    speed.angular.z = rotation * 0.09              #set the rotation speed
    vel_pub.publish(speed)                         #publish the spee
    epsilon = math.pi/(2*10)                       #take 10% of PI/2
    print(goal, yaw_robot)
    
    print("start rotation")
    while not(((goal - epsilon) < yaw) and 
                yaw < (goal + epsilon)):    #while yaw is not close enough of the goal
        #wait
        vel_pub.publish(speed) # I had issues with the publisher : sometimes it was not publying, so i piblish all the time in case
        None

    print("end rotation")
    speed.linear.x = 0          
    speed.angular.z = 0
    vel_pub.publish(speed)      #reset the speed
    return(yaw)

# --------------- other ---------------

def move_to_goal(start,goal): #not used

    # initialise direction to goal and distance to goal
    goal_direction = math.atan2(goal[1]-start[1],goal[0]-start[0])
    goal_dist = math.dist([start[0],start[1]],[goal[0],goal[1]])
    theta = start[2]
    x = start[0]
    y = start[1]
    x_traj, y_traj = [], []
    v = 0.4  # Use to make sure the path is followed at constant speed. Can be changed to slower speeds.

    # stop 10cm from goal. Can be changed to other distances
    while goal_dist > 0.1:
        x_traj.append(x)
        y_traj.append(y)

        x_diff = goal[0] - x
        y_diff = goal[1] - y
        #ensures angles are between -pi and pi (avoids wrapping errors areound 0)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (goal[2] - theta - alpha + np.pi) % (2 * np.pi) - np.pi


        # calcuate velocities
        # DISABLED AS WE WANT THE ROBOT TO FOLLOW AT CONSTANT SPEED.
        # v = Kv*goal_dist
        # ONLY CONTEOLLING DIRECTION IN THIS IMPLEMENTATION.
        w = Ka*alpha+Kb*beta
        #print(v,w)
        # update position
        # THIS WILL NEED TO BE CHANGED BY REAL ROBOT POSITION USING ODOM ON REAL ROBOT
        
        #theta = theta + w * dt
        #x = x + v * np.cos(theta) * dt
        #y = y + v * np.sin(theta) * dt
        #odom version

        x = int(Xrobot/resolution + width/2)
        y = int(Yrobot/resolution + height/2)
        theta = yaw + math.pi/2 #there is an offset of pi/2

        speed.linear.x = v
        speed.angular.z = w
        vel_pub.publish(speed) 
        #print(speed.linear.x)

        goal_dist = math.dist([x,y],[goal[0],goal[1]])
        #print(v,x,y,theta, " and yaw ", yaw + math.pi/2)

        # Display trajectories. Can be removed for real robot

        #if show_animation:  # pragma: no cover
        #    # plt.cla()
        #    plt.arrow(start[0], start[1], np.cos(start[2]),
        #              np.sin(start[2]), color='r', width=0.1)
        #    plt.arrow(goal[0], goal[1], np.cos(goal[2]),
        #              np.sin(goal[2]), color='g', width=0.1)
        #    plot_vehicle(x, y, theta, x_traj, y_traj)
        #add while which turn until robot is close (same condition we made)
        #print("I speed")
        
    
    speed.linear.x = 0
    speed.angular.z = 0
    vel_pub.publish(speed)  #arrived
    
    return x,y,theta

def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def define_map():
    map = np.array(global_map)#create a local variable of the global map, this array is a numpy array
    # Create map on correct format
    
    img = Image.fromarray(np.uint8( map ))                #create and save the map
    img.save('/home/user/catkin_ws/src/assignment_student/pictures/grid_task4.png')
    map = map.astype(float)
    #convert map to correct format for planning 
    for i in range(len(global_map)):        
        for j in range(len(global_map[i])):
            if map[i, j] == 0:          #if the space is free put a 1
                map[i, j] = 1           # freespace
            else:                       #otherwise infinity
                map[i, j] = np.Infinity # obstacle
    return(map)
    
    
def main():

    map = define_map()
    #(map, [Y start, X start], [Y goal, X goal])
    Path = PathPlanning(map, [int(Yrobot/resolution) + 150, int(Xrobot/resolution)+ 150], [150+75 , 150+75]) #+150 is size/2, 
    # Use Dijkstra to find best path to goal
    
    print("Compute path")
    Path.Dijkstra() #use Dijkstra algoritm from PAth planning

    # get and print path
    path = Path.getPath()
    print("path get")
    print(path)
    path = np.array(path)

    # Initialise Path Following parameters
    count = 0
    x_start = path[count][1]
    y_start = path[count][0]
    theta_start = yaw 
    x = x_start 
    y = y_start
    theta = theta_start

    # Follow path feeding each element of the path to the controller.
    while count < len(path)-1:
        x_start = x
        y_start = y
        theta_start = theta
        start = [x,y,theta]
        x_goal = path[count+1][1]
        y_goal = path[count+1][0]
        theta_goal = math.atan2(path[count+1][0]-path[count][0], path[count+1][1]-path[count][1])
        goal = [x_goal,y_goal,theta_goal]
        print(theta_goal, x_goal, y_goal)
        #Use a point and shoot algorithm
        theta = rotation(theta_goal, yaw)   #first, point toward the good direction

        if(path[count+1][0]>path[count][0] or path[count+1][0]<path[count][0]):#look if we move on Y axis
            x, y = translation([x_goal, y_goal], "y")
        else:                                                                  #otherwise,, x axis
            x, y = translation([x_goal, y_goal], "x")
        
        count = count+1
        #rospy.sleep(0.8)
    
    print("OVER")
    speed.linear.x = 0
    speed.angular.z = 0
    vel_pub.publish(speed)  #arrived
    
    




speed = Twist()     #create a Twist object 

if __name__ == '__main__':
    
    rospy.init_node('Assignement1_task4_node')  
    vel_pub = rospy.Publisher('/cmd_vel', Twist ,queue_size=1)              #Velocity publisher 
    odom_sub = rospy.Subscriber('/odom', Odometry, get_position)            #odom subscriber 
    my_map_sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map_callback)#Occupancygrid subscriber

    rospy.sleep(2)                                                          #wait for 2 seconds


    main()                                                                  #call the main function