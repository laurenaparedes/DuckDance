"""lab3 controller."""
# Copyright Prof. Bradley Hayes <bradley.hayes@colorado.edu> 2021
# University of Colorado Boulder CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

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

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
target_pos = ('inf', 'inf')
robot_parts = []


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

# goals = [(1.5, 1), (3, 5)]
goalsX = [1.5, 
            2,
            2,
            1.74, 
            1.5,
            1,
            .25,
            .1,
           .1234]
goalsY = [1, 
            2,
            3,
            3.75,
            4,
            4.5,
            5,
            5.5,
            5.1]

#for each robot we need to add their start and end points from astar
startInMeters = (,)
endInMeters = (,)

start = #need to figure out scaling
end = #need to figure out scaling
#need to load in map from tiago steel
map = np.load("map.npy")

#put into aStar 
path = path_planner(map,start,end)
#from here we need to actually drive along the path
state =0
counter =0
while robot.step(timestep) != -1:
    #Stopping criteria
    if state == len(goalsX)-1:
        robot_parts[MOTOR_LEFT].setVelocity(0)
        robot_parts[MOTOR_RIGHT].setVelocity(0)
        break

    if counter>0:
        counter -=1
        robot_parts[MOTOR_LEFT].setVelocity(0)
        robot_parts[MOTOR_RIGHT].setVelocity(0)
        continue




     #STEP 2: Calculate sources of error
    distanceError = math.sqrt((pose_x - path[state][0])**2 + (pose_y - path[state][1])**2)
    
    bearingError = math.atan2((path[state][1] - pose_y),(path[state][0] - pose_x)) - pose_theta
    if bearingError < -3.1415: bearingError += 6.283
    # if eta < -3.1415: eta += 6.283
   
    headingError = math.atan2((path[state+1][1] - pose_y),(gpath[state+1][0] - pose_x)) - pose_theta
    
    
    if distanceError <.2 and abs(headingError) < 3.1415/4:
        state +=1
        counter =100
        continue
   
    pass 
    
    
    #STEP 2.4: Feedback Controller
    xR = distanceError
    bE = bearingError * 10
    pass
    
    #STEP 1.2: Inverse Kinematics Equations 
    vL = (2 * xR - bE* AXLE_LENGTH) / 2
    vR = (2 * xR + bE* AXLE_LENGTH) / 2
    
    pass
    
    # STEP 2.5: Compute wheel velocities (vL, vR)
    # Already calculated this in step 1.2!
    
    pass

    #STEP 2.7: Proportional velocities
    isnegL = 1
    isnegR = 1
    if vL < 0:
        isnegL = -1
    if vR < 0:
        isnegR = -1
    
    max_val = max(abs(vL), abs(vR), MAX_SPEED)
    vL += max_val
    vR += max_val
    
    pctL = (vL / max(vL, vR) - .5) * 2.
    pctR = (vR / max(vL, vR) - .5) * 2.
   
    vL = pctL * MAX_SPEED
    vR = pctR * MAX_SPEED
   
        

    #STEP 2.6: Clamp wheel speeds
    if vR > MAX_SPEED:
        vR = MAX_SPEED
    
    if vL > MAX_SPEED:
        vL = MAX_SPEED
    
    if vR < -MAX_SPEED:
        vR = -MAX_SPEED
    
    if vL < -MAX_SPEED:
        vL = -MAX_SPEED
    pass

 
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

    