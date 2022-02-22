#!/usr/bin/env python3
import numpy as np
import time
from PIL import Image, ImageDraw
from array import *
import math


# Class for path planning algorithm. It contains an init function to setup parameters and a number of methods to call
# either Dijstra or an A* algorithm. Input parameters are
# - a 2D map filled with either ones for empty cells and Infinity (using np.Infinity) for obstacle cells
# - a start and a goal position

class PathPlanning:

  # Constructor. Checks the map exists and start and goal point are valid
  def __init__(self, map, start, goal):
    self.map = map # map
    if not map.size:
      raise ValueError('Empty map provided')
      return
    self.max_val_x = len(self.map) # x dimension of map
    self.max_val_y = len(self.map[0]) # y dimension of map
    if start[0] not in range(0,self.max_val_x) or start[1] not in range(0 , self.max_val_y) or goal[0] not in range(0, self.max_val_x) or goal[1] not in range(0 , self.max_val_y):
      raise ValueError('Start or Goal Not in range')
      return
    if (map[start[0],start[1]] == np.Infinity) or (map[goal[0],[goal[1]]] == np.Infinity):
      raise ValueError('Start or Goal points not reachable')
      return
    #visualisation parameters
    self.zoom = 20
    self.borders = 6
    self.images = []
    self.start_i , self.start_j = start[0],start[1] #starting point for algorithm
    self.end_i, self.end_j = goal[0],goal[1] #end point for algorithm
    self.path = [] # output path
    self.path_cost = 0
    self.draw = False #parameter to decide to draw in a gif file or not

  def setDraw(self):
    if self.draw == False:
      self.draw = True

  def clearDraw(self):
    if self.draw == True:
      self.draw = False

  # This function saves the path to file
  def savePath(self,fileName):
    if self.draw:
      self.images[0].save(str(fileName),
               save_all=True, append_images=self.images[1:],
               optimize=False, duration=50, loop=0)

  # This function clears the path
  def clearPath(self):
    self.path = []
    self.images = []

  # This function returns the path to follow
  def getPath(self):
    return self.path

  # This function generates the gif file from path
  def draw_matrix(self):
    im = Image.new('RGB', (self.zoom * self.max_val_y, self.zoom * self.max_val_y), (255, 255, 255))
    draw = ImageDraw.Draw(im)
    for i in range(self.max_val_x):
      for j in range(self.max_val_y):
        color = (255, 255, 255)
        r = 0
        if self.map[i][j] == np.Infinity or self.map[i][j]==255:
          color = (0, 0, 0)
        else:
          color = (255*int(self.map[i][j]),255*int(self.map[i][j]),255*int(self.map[i][j]))
        if i == self.start_j and j == self.start_j:
          color = (0, 255, 0)
          r = self.borders
        if i == self.end_i and j == self.end_j:
          color = (0, 255, 0)
          r = self.borders
        draw.rectangle((j * self.zoom + r, i * self.zoom + r, j * self.zoom + self.zoom - r - 1, i * self.zoom + self.zoom - r - 1), fill=color)
    for u in range(len(self.path) - 1):
      y = self.path[u][0] * self.zoom + int(self.zoom / 2)
      x = self.path[u][1] * self.zoom + int(self.zoom / 2)
      y1 = self.path[u + 1][0] * self.zoom + int(self.zoom / 2)
      x1 = self.path[u + 1][1] * self.zoom + int(self.zoom / 2)
      draw.line((x, y, x1, y1), fill=(255, 0, 0), width=5)
    draw.rectangle((0, 0, self.zoom * self.max_val_y, self.zoom * self.max_val_x), outline=(0, 255, 0), width=2)
    self.images.append(im)

  # This calls the Dijkstra algorithm
  def Dijkstra(self):
    #Initialize auxiliary arrays
    distmap=np.ones((self.max_val_x,self.max_val_y),dtype=int)*np.Infinity
    distmap[self.start_i,self.start_j]=0
    originmap=np.ones((self.max_val_x,self.max_val_y),dtype=int)*np.nan
    visited=np.zeros((self.max_val_x,self.max_val_y),dtype=bool)
    finished = False
    x,y=self.start_i,self.start_j
    count=0
    start = time.time()
    #Loop Dijkstra until reaching the target cell
    while not finished:
      # move to x+1,y
      if x < self.max_val_x-1:
        if distmap[x+1,y]>self.map[x+1,y]+distmap[x,y] and not visited[x+1,y]:
          distmap[x+1,y]=self.map[x+1,y]+distmap[x,y]
          originmap[x+1,y]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x-1,y
      if x>0:
        if distmap[x-1,y]>self.map[x-1,y]+distmap[x,y] and not visited[x-1,y]:
          distmap[x-1,y]=self.map[x-1,y]+distmap[x,y]
          originmap[x-1,y]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x,y+1
      if y < self.max_val_y-1:
        if distmap[x,y+1]>self.map[x,y+1]+distmap[x,y] and not visited[x,y+1]:
          distmap[x,y+1]=self.map[x,y+1]+distmap[x,y]
          originmap[x,y+1]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x,y-1
      if y>0:
        if distmap[x,y-1]>self.map[x,y-1]+distmap[x,y] and not visited[x,y-1]:
          distmap[x,y-1]=self.map[x,y-1]+distmap[x,y]
          originmap[x,y-1]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      visited[x,y]=True
      dismaptemp=distmap
      dismaptemp[np.where(visited)]=np.Infinity

      # now we find the shortest path so far
      minpost=np.unravel_index(np.argmin(dismaptemp),np.shape(dismaptemp))
      new_x,new_y=minpost[0],minpost[1]
      if new_x == self.end_i and new_y == self.end_j:
        finished=True
      if new_x == x and new_y == y: # no more progress
        finished =True
        # store last reachable state as closest to goal
        self.end_i = x
        self.end_j = y
        print("imcomplete search")
      else:
        x = new_x
        y = new_y
      count=count+1
    #Start backtracking to plot the path
    x,y=self.end_i,self.end_j
    while x != self.start_i or y != self.start_j:
      self.path.append([int(x),int(y)])
      xxyy=np.unravel_index(int(originmap[int(x),int(y)]), (self.max_val_x,self.max_val_y))
      x,y=xxyy[0],xxyy[1]
      if self.draw:
        self.draw_matrix()

    self.path.append([int(x),int(y)])
    self.path.reverse()  # put path from start to finish rather than finish to start

    if self.draw:
      self.draw_matrix()
    self.path_cost = len(self.path)  #distmap[self.end_i,self.end_j]
    end = time.time()
    print(f"Runtime of Dijkstra search is {end-start}")
    print('The path cost of Dijkstra is: '+str(self.path_cost))

  def A_Star(self):
    #Initialize auxiliary arrays
    distmap=np.ones((self.max_val_x,self.max_val_y),dtype=int)*np.Infinity
    distmap[self.start_i,self.start_j]=0
    originmap=np.ones((self.max_val_x,self.max_val_y),dtype=int)*np.nan
    visited=np.zeros((self.max_val_x,self.max_val_y),dtype=bool)
    finished = False
    x,y=self.start_i,self.start_j
    count=0
    start = time.time()
    #Loop A* until reaching the target cell
    while not finished:
      # move to x+1,y
      if x < self.max_val_x-1:
        if distmap[x+1,y]>self.map[x+1,y]+distmap[x,y]+math.dist([x+1,y],[self.end_i,self.end_j]) and not visited[x+1,y]:
          distmap[x+1,y]=self.map[x+1,y]+distmap[x,y]+math.dist([x+1,y],[self.end_i,self.end_j])
          originmap[x+1,y]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x-1,y
      if x>0:
        if distmap[x-1,y]>self.map[x-1,y]+distmap[x,y]+math.dist([x-1,y],[self.end_i,self.end_j]) and not visited[x-1,y]:
          distmap[x-1,y]=self.map[x-1,y]+distmap[x,y]+math.dist([x-1,y],[self.end_i,self.end_j])
          originmap[x-1,y]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x,y+1
      if y < self.max_val_y-1:
        if distmap[x,y+1]>self.map[x,y+1]+distmap[x,y]+math.dist([x,y+1],[self.end_i,self.end_j]) and not visited[x,y+1]:
          distmap[x,y+1]=self.map[x,y+1]+distmap[x,y]+math.dist([x,y+1],[self.end_i,self.end_j])
          originmap[x,y+1]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      # move to x,y-1
      if y>0:
        if distmap[x,y-1]>self.map[x,y-1]+distmap[x,y]+math.dist([x,y-1],[self.end_i,self.end_j]) and not visited[x,y-1]:
          distmap[x,y-1]=self.map[x,y-1]+distmap[x,y]+math.dist([x,y-1],[self.end_i,self.end_j])
          originmap[x,y-1]=np.ravel_multi_index([x,y], (self.max_val_x,self.max_val_y))
      visited[x,y]=True
      dismaptemp=distmap
      dismaptemp[np.where(visited)]=np.Infinity

      # now we find the shortest path so far
      minpost=np.unravel_index(np.argmin(dismaptemp),np.shape(dismaptemp))
      new_x,new_y=minpost[0],minpost[1]
      if new_x == self.end_i and new_y == self.end_j:
        finished=True
      if new_x == x and new_y == y: # no more progress
        finished =True
        # store last reachable state as closest to goal
        self.end_i = x
        self.end_j = y
        print("imcomplete search")
      else:
        x = new_x
        y = new_y
      count=count+1
    #Start backtracking to plot the path
    x,y=self.end_i,self.end_j
    while x != self.start_i or y != self.start_j:
      self.path.append([int(x),int(y)])
      xxyy=np.unravel_index(int(originmap[int(x),int(y)]), (self.max_val_x,self.max_val_y))
      x,y=xxyy[0],xxyy[1]
      if self.draw:
        self.draw_matrix()

    self.path.append([int(x),int(y)])
    self.path.reverse() # put path from start to finish rather than finish to start
    if self.draw:
      self.draw_matrix()
    self.path_cost = len(self.path)  #distmap[self.end_i,self.end_j]
    end = time.time()
    print(f"Runtime of A* search is {end-start}")
    print('The A* path cost is: '+str(self.path_cost))

