"""turtleGoForward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

if __name__ == "__main__":
# create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.67
    
    #created instances
    part_names = ("left wheel motor", "right wheel motor")
    left_motor = robot.getDevice(part_names[0])
    right_motor = robot.getDevice(part_names[1])
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    num_side = 10
    length_side = 20
    
    wheel_radius = .33
    linear_velocity = wheel_radius * max_speed
    
    duration_side = length_side /linear_velocity
    
    start_time = robot.getTime()
    
    angle_of_rotation = 6.28/num_side
    distance_between_wheels = 0.160
    rate_of_rotation = (2* linear_velocity)/distance_between_wheels
   
    duration_turn = angle_of_rotation/rate_of_rotation +.05
    
    rot_start_time = start_time + duration_side
    rot_end_time = rot_start_time + duration_turn
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        current_time = robot.getTime()
    
        left_speed =  max_speed
        right_speed = max_speed
        
        # print(rot_start_time, current_time, rot_end_time)
        if (rot_start_time < current_time) and (current_time < rot_end_time):
            # print("turning")
            left_speed = max_speed
            right_speed = -max_speed
        elif current_time > rot_end_time:
            rot_start_time = current_time + duration_side
            rot_end_time = rot_start_time + duration_turn
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    # Enter here exit cleanup code.
    