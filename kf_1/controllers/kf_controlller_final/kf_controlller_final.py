"""kf_waypoint1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import time
import numpy

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

flip = False

# Other variables

max_speed = 2 * math.pi 

tp = 0

phi = 0
alpha = 0
theta = 0

state = 0

wheel_radius = 0.025
wheel_distance = 0.11
max_linear_velocity = wheel_radius * max_speed

brake_period = 0.32
steer_period = 0.128
turn_speed = 0.025 * max_speed / 0.22
brake_power = 10

wp_counter = 0
waypoints = [[0.8, 0],\
             [0, 0.8],\
             [-0.8, 0],\
             [0,0],\
             ]
num_points = len(waypoints)

velx = 0
velz = 0

currx = 0
currz = 0

prevdistR = 0
prevdistL = 0

range = 0.16


def setState(destx, destz):

    global tp, state, range 
    
    # print(str(destx) + "\t" + str(destz))
    
    # Check if reached
    if (destx <= range and destx >= -range) \
    and (destz <= range and destz >= -range):
    
        state = 9
        
        print("reached")
    
    # Move straight
    elif (tp >= 3 * brake_period + 4 * steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed \
    + (math.acos(math.cos(theta/2) ** 2) / turn_speed)):
    
        state = 8
        
        tp += timestep / 1000
    
    # Steer wheels straight
    elif (tp >= 3 * brake_period + 3 * steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed \
    + (math.acos(math.cos(theta/2) ** 2) / turn_speed)):
    
        state = 7
        
        tp += timestep / 1000
    
    # Brake third time
    elif (tp >= 2 * brake_period + 3 * steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed \
    + (math.acos(math.cos(theta/2) ** 2) / turn_speed)):
    
        state = 6
        
        tp += timestep / 1000
    
    # Turn right
    elif (tp >= 2 * brake_period + 3 * steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed):
          
          state = 5
          
          tp += timestep / 1000
    
    # Steer wheels right
    elif (tp >= 2 * brake_period + steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed):
        
        state = 4
        
        tp += timestep / 1000
    
    # Brake second time
    elif (tp >= brake_period + steer_period + \
    (math.pi / 2 + theta - math.asin(math.cos(theta / 2) ** 2))/ turn_speed):
        
        state = 3
        
        tp += timestep / 1000
    
    # Turn left
    elif (tp >= brake_period + steer_period):
          
        state = 2
        
        tp += timestep / 1000  
    
    # Steer wheels left
    elif (tp >= brake_period):
    
        state = 1
        
        tp += timestep / 1000
    
    # Brake first time
    elif (tp <= brake_period):
        
        state = 0
       
        tp += timestep / 1000
        
        
               

def moveToWaypoint(destx, destz, currentTime, right_distance, left_distance, yaw):
    
    global theta, state, left_speed, right_speed, front_left_speed, front_right_speed, leftdamp, rightdamp, leftpos, rightpos, tp, flip, wp_counter, num_points
    

    if (tp < timestep / 1000):
        
        # Assign phi
        if yaw < 0:
            
            phi = 2 * math.pi + yaw
            
        else:    
        
            phi = yaw
            
        # Assign alpha
        if destx > 0:
          
              alpha = math.atan(destz / destx)
          
        elif destx < 0:
          
              alpha = math.atan(destz / destx) + math.pi          
    
        else:
            
              alpha = math.atan(destz / 0.001)    
        
        # Assign theta
        if alpha - phi > math.pi:
        
            theta = -(2 * math.pi - (alpha - phi))
           
        elif alpha - phi < -math.pi:
        
            theta = 2 * math.pi + (alpha - phi)
            
        else:
        
            theta = alpha - phi
            
        print(str(theta / math.pi))    
            
        # Alter theta    
        if theta >= 0 and theta <= math.pi:
        
            pass
        
        elif theta >= -math.pi and theta <= 0:
    
            theta = -theta
            flip = True
                    
    setState(destx, destz)
    
    # Brake
    if state == 0: 
        
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0
        
        leftdamp = brake_power
        rightdamp = brake_power
        
    # Steer left
    if state == 1:
    
        leftdamp = 0
        rightdamp = 0
        
        if not flip:
        
            leftpos = math.sin(39.29 / 360 * max_speed)
            rightpos = math.sin(22.25 / 360 * max_speed)
            
        else:
            
            leftpos = math.sin(-22.25 / 360 * max_speed)
            rightpos = math.sin(-39.29 / 360 * max_speed)  
            
    # Turn left
    if state == 2:
        
        if not flip:
        
            left_speed = max_speed / 2
            right_speed = max_speed
        
            front_left_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
            front_right_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22 
        
        else:
        
            left_speed = max_speed
            right_speed = max_speed / 2
        
            front_left_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22
            front_right_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22 
    
    # Brake
    if state == 3:
    
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0 
        
        leftdamp = brake_power
        rightdamp = brake_power
    
    # Steer right
    if state == 4:
    
        leftdamp = 0
        rightdamp = 0
        
        if not flip:
            
            leftpos = math.sin(-22.25 / 360 * max_speed)
            rightpos = math.sin(-39.29 / 360 * max_speed)
    
        else:
        
            leftpos = math.sin(39.29 / 360 * max_speed)
            rightpos = math.sin(22.25 / 360 * max_speed)
                
    # Turn right
    if state == 5:
      
        if not flip:
        
            left_speed = max_speed
            right_speed = max_speed/2
        
            front_left_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22
            front_right_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
        
        else:
        
            left_speed = max_speed / 2
            right_speed = max_speed
        
            front_left_speed = max_speed * (0.09**2 + 0.11**2)**0.5 / 0.22
            front_right_speed = max_speed * (0.09**2 + 0.22**2)**0.5 / 0.22 
        
    # Brake
    if state == 6:
    
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0 
        
        leftdamp = brake_power
        rightdamp = brake_power
    
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
        
        flip = False
    
    # Stop at waypoint and start over    
    if state == 9:
    
        print("bleh")
              
        left_speed = 0
        right_speed = 0
        
        front_left_speed = 0
        front_right_speed = 0 
        
        leftdamp = brake_power
        rightdamp = brake_power
        
        if (wp_counter < num_points - 1):
        
            tp = 0
            wp_counter += 1
        
        
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
    
    # Process sensory input
    
    # IMU standardise yaw
    if roll_pitch_yaw[2] < 0:
            
            roll_pitch_yaw[2] = 2 * math.pi + roll_pitch_yaw[2]
            
    else:    
        
            pass
    
    # Accelaration to velocity
    velx += acc[0] * timestep / 1000
    velz += acc[2] * timestep / 1000
    
    # Odometry to position
    dR = right_distance - prevdistR
    dL = left_distance - prevdistL
    prevdistR = right_distance
    prevdistL = left_distance
    halfcurve = math.asin( dR - dL / 0.22) - roll_pitch_yaw[2]
    
    currx += ((dR + dL) / 2) * math.cos(halfcurve)
    currz += ((dR + dL) / 2) * math.sin(halfcurve)
    
    #print(str(currx) + "\t" + str(-currz) + "\t")
    
    # Update
    
    moveToWaypoint(waypoints[wp_counter][0] - currx, waypoints[wp_counter][1] - -(currz), currentTime, right_distance, left_distance, roll_pitch_yaw[2])
    
    
    # Actuate
    
    #print("roll = " + str(roll_pitch_yaw[0]) + "    pitch = " + str(roll_pitch_yaw[1]) + "    yaw = " + str(roll_pitch_yaw[2]))
    
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


