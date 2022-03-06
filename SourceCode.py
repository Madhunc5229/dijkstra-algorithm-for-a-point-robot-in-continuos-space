from collections import defaultdict
import numpy as np
from cmath import inf
from queue import PriorityQueue
from matplotlib import pyplot as plt
import cv2

#Functions to check if neighbouring points are in obstacle space
def moveLeft(x,y,Map):
    if (0 <= (x-1) < Map.shape[0]) and (0<= (y) < Map.shape[1]):
        if Map[x-1,y] == 0:
            return [x-1, y]
        else:
            return None
    else:
        return None
def moveRight(x,y,Map):
    if (0 <= (x+1) < Map.shape[0]) and (0<= (y) < Map.shape[1]):
        if Map[x+1,y] == 0:
            return [x+1, y]
        else:
            return None
    else:
        return None
def moveDown(x,y,Map):
    if (0 <= (x) < Map.shape[0]) and (0<= (y-1) < Map.shape[1]):
        if Map[x,y-1] ==0:
            return [x, y-1]
        else:
            return None
    else:
        return None
def moveUp(x,y,Map):
    if (0 <= (x) < Map.shape[0]) and (0<= (y+1) < Map.shape[1]):
        if Map[x,y+1] ==0:
            return [x, y+1]
        else:
            return None
    else:
        return None
def moveLeftDiagDown(x,y,Map):
    if (0 <= (x-1) < Map.shape[0]) and (0<= (y-1) < Map.shape[1]):
        if Map[x-1,y-1] ==0:
            return [x-1, y-1]
        else:
            return None
    else:
        return None
def moveLeftDiagUp(x,y,Map):
    if (0 <= (x-1) < Map.shape[0]) and (0<= (y+1) < Map.shape[1]):
        if Map[x-1,y+1] ==0:
            return [x-1, y+1]
        else:
            return None
    else:
        return None
def moveRightDiagDown(x,y,Map):
    if (0 <= (x+1) < Map.shape[0]) and (0<= (y-1) < Map.shape[1]):
        if Map[x+1,y-1] ==0:
            return [x+1, y-1]
        else:
            return None
    else:
        return None
def moveRightDiagUp(x,y,Map):
    if (0 <= (x+1) < Map.shape[0]) and (0<= (y+1) < Map.shape[1]):
        if Map[x+1,y+1] ==0:
            return [x+1, y+1]
        else:
            return None
    else:
        return None

#Function to generate new points (children)
def getNewPoints(point,Map):
    x = point[0]
    y = point[1]
    new_points = []

    if moveLeft(x,y,Map) != None:
        new_points.append(moveLeft(x,y,Map))
    if moveRight(x,y,Map) != None:
        new_points.append(moveRight(x,y,Map))
    if moveDown(x,y,Map) != None:
        new_points.append(moveDown(x,y,Map))
    if moveUp(x,y,Map) != None:
        new_points.append(moveUp(x,y,Map))
    if moveLeftDiagDown(x,y,Map) != None:
        new_points.append(moveLeftDiagDown(x,y,Map))
    if moveLeftDiagUp(x,y,Map) != None:
        new_points.append(moveLeftDiagUp(x,y,Map))
    if moveRightDiagDown(x,y,Map) != None:
        new_points.append(moveRightDiagDown(x,y,Map))
    if moveRightDiagUp(x,y,Map) != None:
        new_points.append(moveRightDiagUp(x,y,Map))

    return new_points

#Function to calculate cost to come between 2 points
def CostToComeBtw(parent, child):
    x_new = child[0]
    y_new = child[1]
    x_old = parent[0]
    y_old = parent[1]

    left = (x_new == x_old-1 and y_new == y_old)

    right = (x_new == x_old+1 and y_new == y_old)

    down = (x_new == x_old and y_new == y_old - 1)

    up = (x_new == x_old and y_new == y_old + 1)

    left_diag_up = (x_new == x_old-1 and y_new == y_old+1)

    left_diag_down = (x_new == x_old-1 and y_new == y_old-1)

    right_diag_up = (x_new == x_old+1 and y_new == y_old+1)

    right_diag_down = (x_new == x_old+1 and y_new == y_old-1)

    if left or right or up or down:
        return 1
    elif left_diag_down or left_diag_up or right_diag_down or right_diag_up:
        return 1.4
    else:
        print("connection doesnt exist")

#BackTracking Function
def backTrack(dict, final_point, start_point, image):

    path = [(final_point[0],final_point[1])]

    i = (final_point[0], final_point[1])

    dict_keys = list(dict.keys())

    dict_values = list(dict.values())
    
    reached_start_point = False

    while not reached_start_point:
        for children in dict_values:
            if i in children:
                image[249-i[1],i[0]] = (255,0,0)
                j = dict_values.index(children)
                path.append(dict_keys[j])
                i = dict_keys[j]
            if (i[0] == start_point[0]) and (i[1] == start_point[1]):
                return path[::-1]

#Get Line equations between two points
def getLine(x1,y1,x2,y2):
    slope = (y2-y1)/(x2-x1)
    y_intercept = (y1 - ((slope)*x1))
    return y-(slope*x) - y_intercept

#Check if the given point is valid (i.e out of obstacle space)
def IsPointValid(x,y,map):
    if map[x,y] == 0:
        return True
    else:
        return False


#Dikkstra Algorithm
def dijkstra(start_point,goal_point,map, CostToCome, image):
    open = PriorityQueue()
    closed = []
    CostToCome[start_point[0],start_point[1]] = 0
    open.put((0,(start_point)))
    parent = defaultdict(list)
    i = 0

    #To record the video 
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter("Dijkstra.mp4", fourcc, 240, (400, 250))

    #Loop through 
    while True:
        #popping from open list and appending to closed
        _, current_point = open.get()
        if current_point not in closed:
            closed.append(current_point)
            print("current point: ",current_point)
        
        #After reaching goal
        if current_point == goal_point:
            print("!!Goal Reached!!")
            print("Video started")
            #start video
            for explored_points in closed:
                image[249-explored_points[1], explored_points[0]] = (255,255,255)
                out.write(image)
                cv2.imshow("exploration", image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            #start backtrack
            backTrack(parent, current_point, start_point, image)

            #Append the backtracked image multiple times so that its visible in the video
            for i in range(0,500):
                out.write(image)
            cv2.imshow("final config",image)
            cv2.waitKey(0)
            break
        
        #Loop to explore new neighbouring points of current point
        for point in getNewPoints(current_point,map):
            
            if point not in closed:
                newC2C = CostToCome[current_point[0],current_point[1]] + CostToComeBtw(current_point, point)
                
                if newC2C < CostToCome[point[0],point[1]]:
                    CostToCome[point[0],point[1]] = newC2C
                    parent[(current_point[0], current_point[1])].append((point[0],point[1]))
                    open.put((CostToCome[point[0],point[1]],point))





if __name__ == '__main__':

    map = np.zeros((400,250))
    image = np.zeros((250,400,3), dtype=np.uint8)
    c2c = np.full((400,250), inf)

    clearance = 5

    #Define obstacle space in map
    for y in range(0,250):
        for x in range(0,400):
            #circle
            if ((x-300)**2) + ((y-185)**2) - ((40+clearance)**2) < 0:
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
                c2c[x,y] = -1
            if ((x-300)**2) + ((y-185)**2) - ((40)**2) < 0:
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                c2c[x,y] = -1
        

            #Lines for traingles with clearance 
            l1_c = getLine(27.49,187.55,136.88,222.17)
            l2_c = getLine(136.88,222.17,85.75,178.35) 
            l3_c = getLine(27.49,187.55,85.75,178.35) 
            l4_c = getLine(27.49,187.55,117.55,76.61)
            l5_c = getLine(85.75,178.35,117.55,76.61) 


            #Defining two traingles from lines
            lower_triangle_c = l4_c >0 and l5_c < 0 and l3_c < 0
            upper_triangle_c = l1_c<0 and l2_c > 0 and l3_c>0

            #Setting the traingles as obstacles
            if  upper_triangle_c :
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
                c2c[x,y] = -1
            if lower_triangle_c :
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
                c2c[x,y] = -1


            #lines for two trianlges without clreance
            l1 = getLine(36,185,115,210)
            l2 = getLine(80,180,115,210)
            l3 = getLine(36,185,80,180) 
            l4 = getLine(36,185,105,100)
            l5 = getLine(80,180,105,100)

            #Defining two traingles from lines without clearance
            lower_triangle = l4 >0 and l5 < 0 and l3 < 0
            upper_triangle = l1<0 and l2 > 0 and l3>0

            #Setting the traingles to 255
            if  upper_triangle :
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                c2c[x,y] = -1
            if lower_triangle :
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                c2c[x,y] = -1
            
            #Defining the lines of hexagon with clearnace
            hexline1_c = getLine(165,120.2,200,140.4) - clearance
            hexline2_c = getLine(200,140.4,235,120.2) - clearance
            hexline5_c = getLine(235,79.8,200,59.6) + clearance
            hexline6_c = getLine(165,79.8,200,59.6) + clearance

            hex_vert_line1_c = x-165 + clearance
            hex_vert_line2_c = x-235 -clearance

            hexagon_c = hexline1_c <0 and hexline2_c <0 and hexline5_c >0 and hexline6_c >0 and hex_vert_line1_c>0 and hex_vert_line2_c <0


            #Setting the hexagon with clearance as obstacle
            if hexagon_c:
                map[x,y] = 1
                image[249-y,x] = (0,0,255)
                c2c[x,y] = -1


            #Defining the triangular lines of hexagon
            hexline1 = getLine(165,120.2,200,140.4) 
            hexline2 = getLine(200,140.4,235,120.2) 
            hexline5 = getLine(235,79.8,200,59.6)
            hexline6 = getLine(165,79.8,200,59.6)

            hex_vert_line1 = x-165
            hex_vert_line2 = x-235

            hexagon = hexline1 <0 and hexline2 <0 and hexline5 >0 and hexline6 >0 and hex_vert_line1>0 and hex_vert_line2 <0


            #Setting the hexagon traingles to 255
            if hexagon:
                map[x,y] = 1
                image[249-y,x] = (255,255,0)
                c2c[x,y] = -1

    #taking input for start point and goal point
    x_start, y_start = int(input("Enter the x cordinate of start node: ")),int(input("Enter the y cordinate of start node: "))
    x_goal, y_goal = int(input("Enter the x cordinate of goal node: ")),int(input("Enter the y cordinate of goal node: "))

    #Checking if the entered input is valid
    while not (IsPointValid(x_start,y_start, map) and IsPointValid(x_goal,y_goal,map)) :
        print("Entered values are in obstacle space. Please enter values that are in free space.")
        x_start, y_start = int(input("Enter the x cordinate of start node: ")),int(input("Enter the y cordinate of start node: "))
        x_goal, y_goal = int(input("Enter the x cordinate of goal node: ")),int(input("Enter the y cordinate of goal node: "))
    
    start_point = [x_start,y_start]
    goal_point = [x_goal,y_goal]
    
    #Calling Dijkstra function to perform search
    dijkstra(start_point, goal_point, map, c2c, image)


    #end
    





