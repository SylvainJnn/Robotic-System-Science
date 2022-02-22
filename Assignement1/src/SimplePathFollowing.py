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

# Control parameters for the PID controller
# These parameters will need to be tuned on real robot
Kv = 9
Ka = 15
Kb = -3
dt = 0.1

show_animation = True

def get_position(info_pos):   
    global Xrobot, Yrobot, roll, pitch, yaw 
    Xrobot = info_pos.pose.pose.position.x
    Yrobot = info_pos.pose.pose.position.y
    
    info_orientation = [info_pos.pose.pose.orientation.x,
                        info_pos.pose.pose.orientation.y,
                        info_pos.pose.pose.orientation.z,
                        info_pos.pose.pose.orientation.w]

    (roll, pitch, yaw) = euler_from_quaternion(info_orientation) #from step :https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/ -- 1000ù c'est juste une fonction voilà maintenaant on connait
    


def move_to_goal(start,goal):

    # initialise direction to goal and distance to goal
    goal_direction = math.atan2(goal[1]-start[1],goal[0]-start[0])
    goal_dist = math.dist([start[0],start[1]],[goal[0],goal[1]])
    theta = start[2]
    x = start[0]
    y = start[1]
    x_traj, y_traj = [], []
    v = 0.5  # Use to make sure the path is followed at constant speed. Can be changed to slower speeds.

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

        x = Xrobot
        y = Yrobot
        theta = yaw + math.pi/2 #there is an offset of pi/2

        speed.linear.x = v
        speed.angular.z = 0
        vel_pub.publish(speed) 

        goal_dist = math.dist([x,y],[goal[0],goal[1]])
        #print(v,x,y,theta, " and yaw ", yaw + math.pi/2)

        # Display trajectories. Can be removed for real robot

        if show_animation:  # pragma: no cover
            # plt.cla()
            plt.arrow(start[0], start[1], np.cos(start[2]),
                      np.sin(start[2]), color='r', width=0.1)
            plt.arrow(goal[0], goal[1], np.cos(goal[2]),
                      np.sin(goal[2]), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)
        #add while which turn until robot is close (same condition we made)
    
    speed.linear.x = 0
    speed.angular.z = 0
    vel_pub.publish(speed)  #arrived
    print("over")
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


def main():
    map = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    ]
    # Create map on correct format
    map = np.array(map)
    plt.imshow(map)
    map = map.astype(float)
    #convert map to correct format for planning #change this to be ok with my map if map[i, j] == 1 then 1 other wise np.Infinity
    for i in range(len(map)):
        for j in range(len(map[i])):
            if map[i, j] == 1:
                map[i, j] = np.Infinity  # obstacle
            else:
                map[i, j] = 1  # free space
    # Create Path planning object
    Path = PathPlanning(map, [1, 1], [5, 19])
    Path.setDraw()
    # Use A-Star to find best path to goal
    Path.A_Star()
    # get and print path
    path = Path.getPath()
    print(path)
    print("I did the path")
    path = np.array(path)

    # Initialise Path Following parameters
    count = 0
    x_start = path[count][1]
    y_start = path[count][0]
    theta_start = math.pi / 2 # change this one -> theta_start = yaw
    x = x_start #odom ? pas forcméent 
    y = y_start
    theta = theta_start

    # Follow path feeding each element of the path to the controller.
    while count < len(path):
        x_start = x
        y_start = y
        theta_start = theta
        start = [x,y,theta]
        x_goal = path[count+1][1]
        y_goal = path[count+1][0]
        theta_goal = math.atan2(path[count+1][0]-path[count][0], path[count+1][1]-path[count][1])
        goal = [x_goal,y_goal,theta_goal]
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
            #(x_start, y_start, theta_start))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
            (x_goal, y_goal, theta_goal))
        print("I print")
        [x,y,theta] = move_to_goal(start,goal)
        count = count+1





if __name__ == '__main__':
    
    rospy.init_node('llllllll')  
    vel_pub = rospy.Publisher('/cmd_vel', Twist ,queue_size=1) 
    odom_sub = rospy.Subscriber('/odom', Odometry, get_position)   
    speed = Twist()
    main()