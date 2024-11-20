"""Robotino_careobot controller."""
from controller import Robot, DistanceSensor, Motor, Keyboard, GPS
import math
import numpy as np

WHEEL_RADIUS = 0.063
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826

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

def ir_read(ir_list):
    irValues = []
    for i in range(len(ir_list)):
        irValues.append(ir_list[i].getValue())
    return irValues
    
def gps_read():
    pass
    return gps.getValues()

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

def navigate(X,Y):
    return 0


# Main loop:
while robot.step(TIME_STEP) != -1:
    
    key=keyboard.getKey()
    if (key==ord('F')):
         print('Ctrl+B is pressed')
    
    #print(ir_read(ir))
    pass
    translate(0, 0, 10)
    print(gps.getValues())
    #forward(5) 

# Enter here exit cleanup code.
