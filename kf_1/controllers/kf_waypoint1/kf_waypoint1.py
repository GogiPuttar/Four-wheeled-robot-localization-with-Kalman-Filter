"""kf_waypoint1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import time

# Create Robot instance
robot = Robot()

# Timestep of current world
timestep = 32


# Device Setup
    
# Rear motors
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
    
# Steering
steer_left_motor = robot.getDevice('steer_left_motor')
steer_right_motor = robot.getDevice('steer_right_motor')
    
# Front motors
front_left_motor = robot.getDevice('front_left_motor')
front_right_motor = robot.getDevice('front_right_motor')
    
# Brakes
left_brake = robot.getDevice('left_brake')
right_brake = robot.getDevice('right_brake')
    
# Steering brakes
steer_left_brake = robot.getDevice('steer_left_brake')
steer_right_brake = robot.getDevice('steer_right_brake')
    
# Odometers
right_pos_sensor = robot.getDevice('right_pos_sensor')
left_pos_sensor = robot.getDevice('left_pos_sensor')
    
# Accelerometer and IMU
accelerometer = robot.getDevice('accelerometer')
imu = robot.getDevice('imu')


# Device initialise

# Rear motors
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# Front motors
front_left_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)

front_right_motor.setPosition(float('inf'))
front_right_motor.setVelocity(0.0)

# Odometers
right_pos_sensor.enable(timestep)
left_pos_sensor.enable(timestep)
    
# Brakes
left_brake.setDampingConstant(0)
right_brake.setDampingConstant(0)

# Steering brakes
steer_left_brake.setDampingConstant(0)
steer_right_brake.setDampingConstant(0)  
    
# Accelerometer and IMU
accelerometer.enable(timestep)
imu.enable(timestep)  
    
# Initialise dynamic variables
    
left_speed = 0
right_speed = 0
        
front_left_speed = 0
front_right_speed = 0

leftpos = 0
rightpos = 0 
    
leftdamp = 0
rightdamp = 0      


# Other variables

max_speed = math.pi * 2

tp = 0

cinch = False

t_b = 0
t_s = 0

dlr = 0
theta = 0

state = 0

wheel_radius = 0.025
wheel_distance = 0.11
max_linear_velocity = wheel_radius * max_speed

brake_period = 0.96
steer_period = 0.128

destx = 30
destz = 40


def setState():

    global tp, t_b, t_s, dlr, theta, cinch, brake_period, steer_period, state 
    
    # Move straight
    if (tp >= 3 * brake_period + 4 * steer_period and state > 4):
    
        state = 8
        
        t_b = 0
        t_s = 0
        tp = 0
    
    # Steer wheels straight
    elif (t_s <= steer_period and tp >= 3 * brake_period + 3 * steer_period and state > 4):
    
        state = 7
        
        t_b = 0
        t_s += timestep / 1000
        tp += timestep / 1000
    
    # Brake third time
    elif (t_b <= brake_period and state > 4 and not(cinch)):
    
        state = 6
        
        t_b += timestep / 1000
        tp += timestep / 1000
        cinch = False
    
    # Turn right
    elif (right_distance - dlr <= \
          0.11 * (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))\
          and tp >= 2 * brake_period + 3 * steer_period and state > 1):
          
          state = 5
          
          t_s = 0
          cinch = True
    
    # Steer wheels right
    elif (t_s <= 2 * steer_period and tp >= 2 * brake_period + steer_period \
          and state > 1 and not(cinch)):
        
        state = 4
        
        t_b = 0
        t_s += timestep / 1000
        tp += timestep / 1000
        dlr = right_distance
    
    
    # Brake second time
    elif (t_b <= brake_period and state > 1 and not(cinch)):
        
        state = 3
        
        t_b += timestep / 1000
        tp += timestep / 1000
        dlr = 0
        cinch = False
    
    # Turn left
    elif (left_distance - dlr <= \
          0.11 * (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))\
          and tp >= brake_period + steer_period and state > 0):
          
          state = 2
          
          print(str(left_distance ))
          t_s = 0
          cinch = True
    
    # Steer wheels left
    elif (t_s <= steer_period and tp >= brake_period and not(cinch)):
    
        state = 1
        
        t_b = 0
        t_s += timestep / 1000
        tp += timestep / 1000
        dlr = right_distance
    
    # Brake first time
    elif (t_b <= brake_period and not(cinch)):
        
        state = 0
        
        t_b += timestep / 1000
        tp += timestep / 1000
    
    # Release cinch
    else:
        
        if (state == 5):
            
            state = 6
            
        elif (state == 2):
            
            state = 3    
            
        t_b += timestep / 1000
        tp += timestep / 1000
        cinch = False       

def moveToWaypoint(destx, destz, currentTime, right_distance, left_distance):
    
    global theta, state, left_speed, right_speed, front_left_speed, front_right_speed, leftdamp, rightdamp, leftpos, rightpos
    
    theta = math.atan(destz/destx) - 0
    
    setState()
    
    print(str(state))
    
    # Brake
    if state == 0: 
        
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0
        
        leftdamp = 100
        rightdamp = 100
        
    # Steer left
    if state == 1:
    
        leftpos = math.sin(39.29 / 360 * max_speed)
        rightpos = math.sin(22.25 / 360 * max_speed)
            
    # Turn left
    if state == 2:
        
        left_speed = max_speed/2
        right_speed = max_speed
        
        front_left_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
        front_right_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22 
        
    # Brake
    if state == 3:
    
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0 
        
        leftdamp = 100
        rightdamp = 100
    
    # Steer right
    if state == 4:
    
        leftdamp = 0
        rightdamp = 0
        
        leftpos = math.sin(-22.25 / 360 * max_speed)
        rightpos = math.sin(-39.29 / 360 * max_speed)
    
    # Turn right
    if state == 5:
      
        left_speed = max_speed
        right_speed = max_speed/2
        
        front_left_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22
        front_right_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
    
    # Brake
    if state == 6:
    
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0 
        
        leftdamp = 100
        rightdamp = 100
    
    # Make wheels straight
    if state == 7:
    
        leftdamp = 0
        rightdamp = 0
        
        leftpos = math.sin(0)
        rightpos = math.sin(0)
    
    # Move straight
    if state == 8:
    
        left_speed = max_speed
        right_speed = max_speed
        
        front_left_speed = max_speed
        front_right_speed = max_speed
        
# Initialize Start time    
startTime = robot.getTime()


while robot.step(timestep) != -1:
    
    currentTime = robot.getTime() - startTime
    
    # Sense 
    
    # Odometer
    right_distance = -right_pos_sensor.getValue() * wheel_radius
    left_distance = -left_pos_sensor.getValue() * wheel_radius
    
    # Accelerometer and IMU
    acc = accelerometer.getValues()
    roll_pitch_yaw = imu.getRollPitchYaw()
    quaternion = imu.getQuaternion()
    
    
    # Update
    
    moveToWaypoint(destx, destz, currentTime, right_distance, left_distance)
    
    
    # Actuate
    
    # Rear wheels velocity
    left_motor.setVelocity(-left_speed)
    right_motor.setVelocity(-right_speed)
        
    # Front wheels velocity
    front_left_motor.setVelocity(-front_left_speed)
    front_right_motor.setVelocity(-front_right_speed)
    
    # Steering motors position
    steer_left_motor.setPosition(leftpos)
    steer_right_motor.setPosition(rightpos)
    
    # Brakes
    left_brake.setDampingConstant(leftdamp)
    right_brake.setDampingConstant(rightdamp)
    
    pass

# Enter here exit cleanup code.


