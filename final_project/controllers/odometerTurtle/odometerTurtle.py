"""odometerTurtle controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
max_speed = 6.67
    
    #created instances
part_names = ("left wheel motor", "right wheel motor")
left_motor = robot.getDevice(part_names[0])
right_motor = robot.getDevice(part_names[1])
    
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
    
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

left_ps = robot.getDevice('left wheel sensor')
left_ps.enable(timestep)

right_ps = robot.getDevice('right wheel sensor')
right_ps.enable(timestep)

ps_values = [0,0]
wheel_radius = .33
distance_between_wheels = 0.160


wheel_cir = 2 * 3.14 * wheel_radius
encoder = wheel_cir/6.28

robot_pose = [0,0,0]#x, y, theta
last_ps_values =[0,0]

num_side = 5
length_side = 8
    
wheel_radius = .33
linear_velocity = wheel_radius * max_speed
    
duration_side = length_side /linear_velocity
    
start_time = robot.getTime()
    
angle_of_rotation = 6.28/num_side
rate_of_rotation = (2* linear_velocity)/distance_between_wheels
   
duration_turn = angle_of_rotation/rate_of_rotation +.03
    
rot_start_time = start_time + duration_side
rot_end_time = rot_start_time + duration_turn
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
dist_values =[0,0]
count = 0
while robot.step(timestep) != -1 and num_side>count:
    
    current_time = robot.getTime()
    
    left_speed = max_speed
    right_speed = max_speed
    
    if (rot_start_time < current_time) and (current_time < rot_end_time):
        # print("turning")
        left_speed = max_speed
        right_speed = -max_speed
    elif current_time > rot_end_time:
        
        rot_start_time = current_time + duration_side
        rot_end_time = rot_start_time + duration_turn
    if current_time == rot_end_time:
            count+=1
    
    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()
    
    for i in range(2):
        diff = ps_values[i] - last_ps_values[i]
        if diff <.001:
            diff = 0
            ps_values[i]= last_ps_values[i]
        dist_values[i] = diff*encoder
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    v =(dist_values[0] +dist_values[1])/2
    w = (dist_values[0] +dist_values[1])/ distance_between_wheels
    
    dt = 1
    robot_pose[2] += w*dt
    
    vx = v * math.cos(robot_pose[2])
    vy = v * math.sin(robot_pose[2])
    
    robot_pose[0] += vx *dt
    robot_pose[1] += vy *dt
    
    
    for i in range(2):
        last_ps_values[i] = ps_values[i]
    
    
    
# Enter here exit cleanup code.
