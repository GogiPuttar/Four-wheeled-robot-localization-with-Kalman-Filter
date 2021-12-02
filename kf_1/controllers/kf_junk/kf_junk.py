"""drive_my_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import time

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32

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
right_pos_sensor = robot.getDevice('right_pos_sensor')
left_pos_sensor = robot.getDevice('left_pos_sensor')


max_speed = math.pi * 2
tr = 0
tl = 0


left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

front_left_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)

front_right_motor.setPosition(float('inf'))
front_right_motor.setVelocity(0.0)

right_pos_sensor.enable(timestep)
left_pos_sensor.enable(timestep)

steer_left_brake.setDampingConstant(0)

steer_right_brake.setDampingConstant(0)


wheel_radius = 0.025
wheel_distance = 0.11
max_linear_velocity = wheel_radius * max_speed

startTime = robot.getTime()

            
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    currentTime = robot.getTime() - startTime
    
    right_distance = -right_pos_sensor.getValue() * wheel_radius
    left_distance = -left_pos_sensor.getValue() * wheel_radius
    
    # print(str(right_pos_sensor.getValue()))
    
    if currentTime <= 1:   
    
            leftpos = math.sin(39.29 / 360 * max_speed)
    
            rightpos = math.sin(22.25 / 360 * max_speed)

    # print(str(duration_side) + "   " + str(duration_turn))
    else: 
        
        print("now")
        left_speed = max_speed / 2
        right_speed = max_speed
        
        front_left_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
        front_right_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22
        
    
        left_motor.setVelocity(-left_speed)
        right_motor.setVelocity(-right_speed)
        
        front_left_motor.setVelocity(-front_left_speed)
        front_right_motor.setVelocity(-front_right_speed)
        
        #steer_left_brake.setDampingConstant(100)
        #steer_right_brake.setDampingConstant(100)
     
    steer_left_motor.setPosition(leftpos)
    steer_right_motor.setPosition(rightpos)
        
    print("Right: " + str(right_distance) + "    Left: " + str(left_distance) + "    Time: " + str(currentTime)) 
        
    
   
    #print(str(i) + "~" + str(robot.step(timestep)))
    
    # print("heyy")
    
    pass

# Enter here exit cleanup code.