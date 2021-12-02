"""drive_my_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 1
i = 0

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
steer_left_motor = robot.getDevice('steer_left_motor')
steer_right_motor = robot.getDevice('steer_right_motor')


max_speed = math.pi * 2


left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

steer_left_motor.setPosition(float('inf'))
steer_left_motor.setVelocity(0.0)

steer_right_motor.setPosition(float('inf'))
steer_right_motor.setVelocity(0.0)


side_length = 0.25

wheel_radius = 0.025
wheel_distance = 0.11
max_linear_velocity = wheel_radius * max_speed

duration_side = side_length / max_linear_velocity
duration_turn = math.pi/2 * 0.2377 / max_linear_velocity
duration_break = 1
startTime = robot.getTime()


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # print(str(duration_side) + "   " + str(duration_turn))
    if( robot.getTime() - startTime >= duration_side + duration_turn + 2 * duration_break) :
        
        left_speed = max_speed
        right_speed = max_speed    
        startTime = robot.getTime()
        
    elif( robot.getTime() - startTime >= duration_side + duration_turn + duration_break) :
           
        left_speed = 0
        right_speed = 0
        
    elif( robot.getTime() - startTime >= duration_side + duration_break) :
        
        left_speed = -max_speed
        right_speed = max_speed
        
    elif( robot.getTime() - startTime >= duration_side) :
        
        left_speed = - 0
        right_speed = 0    
    
    else:
    
        left_speed = max_speed
        right_speed = max_speed  
  
    
    left_motor.setVelocity(-left_speed)
    right_motor.setVelocity(-right_speed)
    
    
    #print(str(i) + "~" + str(robot.step(timestep)))
    
    i = i + 1
    
    # print("heyy")
    
    pass

# Enter here exit cleanup code.