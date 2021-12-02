"""drive_my_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import time

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
front_left_motor = robot.getDevice('front_left_motor')
front_right_motor = robot.getDevice('front_right_motor')
steer_left_brake = robot.getDevice('steer_left_brake')
steer_right_brake = robot.getDevice('steer_right_brake')


max_speed = math.pi * 2


left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

#steer_left_motor.setPosition(float('inf'))
steer_left_motor.setVelocity(0.0)

#steer_right_motor.setPosition(float('inf'))
#steer_right_motor.setVelocity(0.0)

front_left_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)

front_right_motor.setPosition(float('inf'))
front_right_motor.setVelocity(0.0)

steer_left_brake.setDampingConstant(0)

steer_right_brake.setDampingConstant(0)


side_length = 0.25

wheel_radius = 0.025
wheel_distance = 0.11
max_linear_velocity = wheel_radius * max_speed

duration_side = side_length / max_linear_velocity
duration_turn = math.pi/2 * 0.2377 / max_linear_velocity
duration_break = 1
val = math.pi 

startTime = robot.getTime()

            
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    currentTime = robot.getTime() - startTime
    
    if currentTime <= 3:
        if currentTime <= 39.29 / 360:   
            steer_left_motor.setPosition(val)
            #steer_left_motor.setVelocity(max_speed)
        else:
            steer_left_motor.setVelocity(0)
            #steer_left_brake.setDampingConstant(100)
    
    
        if currentTime <= 22.25 / 360:
            steer_right_motor.setPosition(val)
            #steer_right_motor.setVelocity(max_speed)
        else:
            steer_right_motor.setVelocity(0)
            #steer_left_brake.setDampingConstant(100)

    # print(str(duration_side) + "   " + str(duration_turn))
    elif currentTime <= 1000: 
        
        left_speed = max_speed / 2
        right_speed = max_speed
        
        front_left_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
        front_right_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22
        
        steer_left_speed = 0
        steer_right_speed = 0
        
    
        left_motor.setVelocity(-left_speed)
        right_motor.setVelocity(-right_speed)
        
        front_left_motor.setVelocity(-front_left_speed)
        front_right_motor.setVelocity(-front_right_speed)
        
        steer_left_motor.setVelocity(steer_left_speed)
        steer_right_motor.setVelocity(steer_right_speed)
        
        #steer_left_brake.setDampingConstant(100)
        #steer_right_brake.setDampingConstant(100)
        steer_left_motor.setPosition(val)
        steer_right_motor.setPosition(val)
    else:
        
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        front_left_motor.setVelocity(0)
        front_right_motor.setVelocity(0)
        
        #steer_left_motor.setVelocity(0)
        #steer_right_motor.setVelocity(0)
        
        time.sleep(4)
        steer_left_motor.setPosition(0)
        steer_right_motor.setPosition(0)
    
    
    
    #print(str(i) + "~" + str(robot.step(timestep)))
    
    i = i + 1
    
    # print("heyy")
    
    pass

# Enter here exit cleanup code.