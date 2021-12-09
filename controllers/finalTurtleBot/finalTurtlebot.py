"""lab3 controller."""
# Copyright Prof. Bradley Hayes <bradley.hayes@colorado.edu> 2021
# University of Colorado Boulder CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard, CameraRecognitionObject, GPS
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
# from ikpy.chain import Chain
# from ikpy.link import OriginLink, URDFLink

# https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.160 # [m]
WHEEL_RADIUS = 0.33

MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

compass = robot.getDevice("compass")
compass.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
target_pos = ('inf', 'inf')
robot_parts = []

class Node():
    """A node class for A* Pathfinding"""
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def path_planner(map, start, end):
        '''
        # A* code adapted from
        # https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell
        :param end: A tuple of indices representing the end cell
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:
            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0),
                                 (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > len(map) - 1 or node_position[0] < 0 or node_position[1] > \
                        len(map[len(map) - 1]) - 1 or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if map[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                if new_node not in closed_list:
                    children.append(new_node)

            # Loop through children
            for child in children:
                # Create the f, g, and h values
                if abs(child.position[0] - current_node.position[0]) + abs(
                        child.position[1] - current_node.position[1]) == 2:
                    # If the child is diagonally placed then the distance is math.sqrt(2)x grid_size
                    # grid_size is 1 in this case
                    child.g = current_node.g + math.sqrt(2)
                else:
                    child.g = current_node.g + 1
                # H: Euclidean distance to end point
                child.h = math.sqrt(
                    (child.position[0] - end_node.position[0]) ** 2 + (child.position[1] - end_node.position[1]) ** 2)
                # child.h=0 # h=0 makes this Dijkstra's
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    # check if the new path to children is worst or equal
                    # than one already in the open_list (by measuring g)
                    if child == open_node:
                        if child.g >= open_node.g:
                            break
                        else:
                            open_node.g = child.g
                            open_node.parent = child.parent
                else:
                    # Add the child to the open list
                    open_list.append(child)

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0


startInMeters = (3.9,11.2)
endInMeters = (5,8.8)

start = (int(startInMeters[0] * 30), int(startInMeters[1] *30))
end = (int(endInMeters[0] * 30), int(endInMeters[1] *30))
#need to load in map from tiago steel
#map = np.load("convolved_map.npy")

#put into aStar 
#path = path_planner(map,start,end)
#print(path)
#from here we need to actually drive along the path

loadMap = np.load('map.npy')

# Part 2.2: Compute an approximation of the “configuration space”
kernel = np.ones((10, 10)) # Play with this number to find something suitable
convolved_map = convolve2d(loadMap, kernel, mode ='same') # You still have to threshold this convolved map
    
convolved_map = np.multiply(convolved_map >0.5,1)
        
# fig = plt.figure(figsize=(12, 8), dpi=100, facecolor='w', edgecolor='k')
# plt.imshow(convolved_map)
# plt.show()
       
    # th, start_w = cv2.threshold(convolved_map, thresholded_map, 0, 255, cv2.THRESH_BINARY)

        # create a temp copy of the map array
        # for every 1 in the map array in the temp one make all surrounding blocks a 1 as well (like a square around)
        # repeat that a couple times to make the squares “denser”
    
# Part 2.3 continuation: Call path_planner
# np.save("convolved_map", convolved_map)
    
path = path_planner(convolved_map, start, end)
waypoints = []
for p in path:
    g = ((p[0]) / 30, p[1] / 30)
    waypoints.append(g)
np.save("path", waypoints)
        
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    # np.save("path", path)
waypoints = np.load('path.npy')
# for p in path: convolved_map[p[0]][p[1]] = 2
# fig = plt.figure(figsize=(12, 8), dpi=100, facecolor='w', edgecolor='k')
# plt.imshow(convolved_map)
# plt.show()

x_values=[i[0] for i in waypoints]
y_values = [i[1] for i in waypoints]
state =0
counter =0
# print(waypoints[0], waypoints[1])/
while robot.step(timestep) != -1:
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
    dX = 0
    dTheta = 0
        #STEP 1: Calculate the error
    rho = math.sqrt((waypoints[state][0] - pose_x)**2 +(waypoints[state][1] - pose_y)**2)
    alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)
    # print(waypoints[state][0], waypoints[state][1])
    # print(pose_x, pose_y)
    # print(rho, alpha)    
        #STEP 2: Controller
    if state == len(x_values)-1:
            #print('autonomous if state')
        robot_parts[MOTOR_LEFT].setVelocity(0)
        robot_parts[MOTOR_RIGHT].setVelocity(0)
        break

        # bearingError = math.atan2((y_values[state] - pose_y),(x_values[state] - pose_x)) - pose_theta
    
    print("Abs Alpha: {:.3f}".format(abs(alpha)))
    print("rho: {:.3f}".format(rho))
    if abs(alpha)>.5:
         
        dX = 0
        dTheta = 10*alpha
    else:
        dX = 10*rho
        dTheta = 5*alpha
        
    if rho <.6:
            #print('distanceError') 
        state +=1
            # counter =100
        continue
    print("DX: {:.3f}".format(dX))
    print("dTheta: {:.3f}".format(dTheta))
        #STEP 3: Compute wheelspeeds
    vL = (dX - dTheta * AXLE_LENGTH/2)
    vR = (dX + dTheta * AXLE_LENGTH/2)
       
    if vL > MAX_SPEED:
        
        vL = MAX_SPEED
    if vR > MAX_SPEED:
        vR = MAX_SPEED
        
    if vL < -MAX_SPEED :
        vL = -MAX_SPEED
    if vR < -MAX_SPEED:
        vR = -MAX_SPEED
        
    # Odometry code. Don't change speeds (vL and vR) after this line
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    
    # TODO: Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)