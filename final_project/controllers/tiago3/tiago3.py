"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard, CameraRecognitionObject
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
# from ikpy.chain import Chain
# from ikpy.link import OriginLink, URDFLink


# import pybullet as p
# import pybullet_data
# import cv2

# with open("tiago_urdf.urdf", "w") as file:  
    # file.write(robot.getUrdf())

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# create camera instance
camera = Camera(name = 'camera')
cameraRecognitionObject = CameraRecognitionObject()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
# mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
mode = 'autonomous'


###################
#
# Planner
#
###################
if mode == 'planner':
    start_w = (14.276768, 26.6914) # Pose_X, Pose_Z in meters
    end_w = (9.41594, 23.94844) # Pose_X, Pose_Z in meters

    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = (int(start_w[0]*30), int(start_w[1]*30)) # (134, 241) # (x, y) in 360x360 map
    end = (int(end_w[0]*30), int(end_w[1]*30)) # (300, 210) # (x, y) in 360x360 map

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

# Part 2.3: Implement A* or Dijkstra's
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
    
    # Part 2.1: Load map (map.npy) from disk and visualize it
    loadMap = np.load('map.npy')

    # Part 2.2: Compute an approximation of the “configuration space”
    kernel = np.ones((15, 15)) # Play with this number to find something suitable
    convolved_map = convolve2d(loadMap, kernel, mode ='same') # You still have to threshold this convolved map
    
    convolved_map = np.multiply(convolved_map >0.5,1)
        
    fig = plt.figure(figsize=(12, 8), dpi=100, facecolor='w', edgecolor='k')
    plt.imshow(convolved_map)
    plt.show()
       
    # th, start_w = cv2.threshold(convolved_map, thresholded_map, 0, 255, cv2.THRESH_BINARY)

        # create a temp copy of the map array
        # for every 1 in the map array in the temp one make all surrounding blocks a 1 as well (like a square around)
        # repeat that a couple times to make the squares “denser”
    
    # Part 2.3 continuation: Call path_planner
    np.save("convolved_map", convolved_map)
    # print("saved CM")
    # print(convolved_map)
    path = path_planner(convolved_map, start, end)
    # print("path finished")
    waypoints = []
    # print(path)
    for p in path:
        # print(path[p])
        g = ((p[0]) / 30, p[1] / 30)
        waypoints.append(g)
    np.save("path", waypoints)
        
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    # np.save("path", path)
    waypoints = np.load('path.npy')
    # print("saved path")
    for p in path: convolved_map[p[0]][p[1]] = 2
    fig = plt.figure(figsize=(12, 8), dpi=100, facecolor='w', edgecolor='k')
    plt.imshow(convolved_map)
    plt.show()

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
map = np.zeros((900, 900), dtype = np.float64) # Replace None by a numpy 2D floating point array
waypoints = []


if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = np.load('path.npy') # Replace with code to load your path
    
    
    for i in range(8):
        np.delete(waypoints, -1)# plt.imshow(waypoints)
    # print(waypoints)
    x_values=[i[0] for i in waypoints]
    y_values = [i[1] for i in waypoints]
    
    state = 0 # use this to iterate through your path

while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2] + 15
    pose_x = gps.getValues()[0] + 15

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho 
        ry = -math.sin(alpha)*rho 

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x 
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y 
    
        ################ ^ [End] Do not modify ^ ##################

        # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            
            # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
           
            if int(wx * 30) > 899 or int(wx * 30) < 0:
                pass
            
            elif int(wy * 30) > 899 or int(wy * 30) < 0:
                pass
            
            else:
            
                map[int(wx*30)][int(wy*30)] += 0.005
                
                if map[int(wx*30)][int(wy*30)] > 1:
                    map[int(wx*30)][int(wy*30)] = 1
                
                g = int(map[int(wx*30)][int(wy*30)] * 255) # converting [0,1] to grayscale intensity [0,255]
                color = (g*256**2+g*256+g)
                
                display.setColor(color)
                display.drawPixel(900-int(wy*30),int(wx*30))
    
    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(900-int(pose_y*30),int(pose_x*30))
    
    display.setColor(0x00FF00)
    path_on_map = []
    for p in waypoints:
        p_on_map = (int(p[0]*30), int(p[1]*30))
        path_on_map.append(p_on_map)
    for i in range(len(path_on_map)):
        if i > 0:
            display.drawLine(900-path_on_map[i-1][1],path_on_map[i-1][0],900-path_on_map[i][1],path_on_map[i][0])

    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            
            # Part 1.4: Filter map and save to filesystem
            # filesystem = []
            
            filesystem = map > 0.5
            map = np.multiply(map > 0.5,1)
            np.save("map",map)
            print("Map file saved")
        
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        # Part 3.2: Feedback controller
        dX = 0
        dTheta = 0
        #STEP 1: Calculate the error
        rho = math.sqrt((waypoints[state][0] - pose_x)**2 +(waypoints[state][1] - pose_y)**2)
        alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)
        
        #STEP 2: Controller
        if state == len(x_values)-1:
            #print('autonomous if state')
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)
            break

        # bearingError = math.atan2((y_values[state] - pose_y),(x_values[state] - pose_x)) - pose_theta
        if abs(alpha)>0.25:
            dX = 0
            dTheta = 10*alpha
        else:
            dX = 10*rho
            dTheta = 5*alpha
        
        if rho <.4:
            #print('distanceError') 
            state +=1
            # counter =100
            continue

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
        
    
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta) 
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta) 
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)