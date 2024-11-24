"""Robotino_careobot controller."""
from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Compass
import math
import numpy as np
from simple_pid import PID

WHEEL_RADIUS = 0.063
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
distance_threshold = 0.05
translation_speed = 10

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# initialize devices
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# IR Sensors: ir1(12oclock), ir2(11oclock), ... , ir9(1oclock) (clock from top view)
# Bumper Touch Sensor: touch sensor
# Compass: compass
# Gyro: gyro
# IMU: inertial unit
# Accelerometer: accelerometer
# Motors:  wheel0_joint(back), wheel1_joint(rignt), wheel2_joint(left)
# wheel0_joint_sensor

# IR Sensors Initialize
ir = []
irNames = ['ir1','ir2','ir3','ir4','ir5','ir6','ir7','ir8','ir9',]
for i in range(9):
    ir.append(robot.getDevice(irNames[i]))
    ir[i].enable(TIME_STEP)

# Motor Initialize
Lmotor = robot.getDevice('wheel2_joint')
Rmotor = robot.getDevice('wheel1_joint')
Bmotor = robot.getDevice('wheel0_joint')

Lodo = robot.getDevice('wheel2_joint_sensor').enable(TIME_STEP)
Rodo = robot.getDevice('wheel1_joint_sensor').enable(TIME_STEP)
Bodo = robot.getDevice('wheel0_joint_sensor').enable(TIME_STEP)

Lmotor.setPosition(float('inf'))
Rmotor.setPosition(float('inf'))
Bmotor.setPosition(float('inf'))

Lmotor.setVelocity(0.0)
Rmotor.setVelocity(0.0)
Bmotor.setVelocity(0.0)


gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

def ir_read(ir_list):
    irValues = []
    for i in range(len(ir_list)):
        irValues.append(ir_list[i].getValue())
    return irValues
    
def gps_read():
    pass
    return gps.getValues()

def bearing():
    compX , compY , compZ = compass.getValues()
    theta = math.atan2(compY , compX) 
    bearing = math.degrees(theta)
    if bearing < 0.0:
        pass
        bearing = bearing + 360.0
    bearing = 360 - bearing
    return bearing

def translate(deg, omega, Speed=1, distance='inf'):
    rad = math.radians(deg+30)
    r = WHEEL_RADIUS
    d = DISTANCE_WHEEL_TO_ROBOT_CENTRE
    omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS
    
    Vx = Speed * math.cos(rad)
    Vy = Speed * math.sin(rad)
    omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS
    
    A = np.array([[1,-0.5,-0.5],[0,-math.sqrt(3)/2, math.sqrt(3)/2],[1,1,1]])
    B = np.array([Vx,Vy,omega])
    C = np.linalg.solve(A,B)
    
    """
    Vx /= WHEEL_RADIUS
    Vy /= WHEEL_RADIUS
    omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS
    
    V_L = math.sqrt(0.75) * Vx - 0.5 * Vy - omega
    V_R = - math.sqrt(0.75) * Vx - 0.5 * Vy - omega
    V_B = Vy - omega"""
    
    #print(Vx,Vy)
    
    Lmotor.setVelocity(C[0])
    Rmotor.setVelocity(C[1])
    Bmotor.setVelocity(C[2])

def navigate(Y1,X1,Y2,X2):
    try:
        delta_x = X2 - X1
        delta_y = Y2 - Y1
        distance_to_target = math.sqrt(delta_x**2 + delta_y**2)
        theta = math.atan2(delta_y, delta_x)  # atan2 automatically handles signs
        if theta < 0.0:
            #theta = theta + (2 * math.pi)
            pass
        degree = math.degrees(theta)
        degree = degree - bearing()
        if distance_to_target < distance_threshold:
            speed = min(translation_speed, distance_to_target * 5)  # Scaled speed
        else: speed = translation_speed
        
        translate(degree,0,speed)
        
        #print(degree, theta)
    except ZeroDivisionError:
        pass


# Main loop:
while robot.step(TIME_STEP) != -1:
    
    key=keyboard.getKey()
    if (key==ord('F')):
         print('Ctrl+B is pressed')
    
    #print(ir_read(ir))
    pass
    #translate(-90, 0, 10)
    
    print(bearing())
    gpsX , gpsY , gpsZ = gps.getValues()
    gpsX = round(gpsX * 100000 , 8)
    gpsY = round(gpsY * 100000 , 8)
    print("       ",gpsX, " , ",gpsY)
    navigate(gpsX,gpsY,0.0,0.5)
 

# Enter here exit cleanup code.
