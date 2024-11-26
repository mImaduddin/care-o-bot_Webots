"""Robotino_careobot controller."""
from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Compass
import math
import pickle
import pygame
import numpy as np
from simple_pid import PID
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from time import sleep


WHEEL_RADIUS = 0.063
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826

distance_threshold = 10
translation_speed = 10


grid_file_address = 'D:\\OneDrive - Ostbayerische Technische Hochschule Amberg-Weiden\\My Projects\\Webots_Robotino\\grid_data.pkl'



ROOM_X = 10 # meters
ROOM_Y = 10 # meters
WIDTH = 600
ROWS = 50 # The larger the rows the slower the calculation
GPS_ORIGIN = [4.51800361  ,  -4.4876655]
HOME_POS = [3.10810397  ,  -2.95265777]


#WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Robotino Path Finding Algorithm")

RED = (255, 100, 100)
GREEN = (100, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (50, 0, 255)
ORANGE = (255, 165 ,0)
GREY = (200, 200, 200)
TURQUOISE = (64, 224, 208)



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
# Gps: Gps
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



class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_empty(self):
        return self.color != BLACK

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width)) # this draws a single box


def make_grid(rows, width): # this function creates new "Spots instances" for the grid, spits out an instantiated grid object
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid # each element has an instance of the Spot class

def load_grid_file(file):
    with open(file, "rb") as file:
        loaded_grid = pickle.load(file)
        print("LOADED: ")
    return loaded_grid

def save_grid_file(objects, file):
    with open(file, "wb") as file:
        pickle.dump(objects,file)
        print("SAVED: ")

def draw(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win) # actually draws the rectangles stored in thr grid.(whatever their atribs are)

    draw_grid_lines(win, rows, width) # draw grid lines over it
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col

def draw_grid_lines(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))



def edit_grid_window(win, width):
    grid = make_grid(ROWS, width) # it makes its new grid, completely blank
    start = None
    end = None

    run = True
    while run and robot.step(TIME_STEP) != -1:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]: # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col] # here we figured out thr spot clicked
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]: # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and start and end:
                    run = False # return the grid

                    #algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()
    return grid

def show_grid_window(win, grid, width):
    run = True
    while run and robot.step(TIME_STEP) != -1:
        draw(win, grid, ROWS, width)
        edit_path(path,grid)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
    pygame.quit()   

def start_node(grid):
    for rows in grid: # Finding the Start Node
        for obj in rows:
            if obj.is_start():
                node = [obj.row,obj.col]
    return node

def end_node(grid):
    for rows in grid: # Finding the End Node
        for obj in rows:
            if obj.is_end():
                node = [obj.row,obj.col]
    return node

def algorithm(grid, start, end):
    grid_matrix = [[row[i].is_empty() for row in grid] for i in range(len(grid[0]))] # convert grid to boolean or numeric matrix
    #print(grid_matrix)
    grid = Grid(matrix=grid_matrix)
    start_node_ = grid.node(start[0],start[1])
    end_node_ = grid.node(end[0],end[1])
    finder = AStarFinder(diagonal_movement = DiagonalMovement.always)
    path_obj , runs = finder.find_path(start_node_, end_node_, grid)

    path = [[obj.x,obj.y] for obj in path_obj] # NOW YOU HAVE THE PATH!!!!
    #print(path)
    return path


    return path

def edit_path(path, grid):
    for i in path:
        row = i[0]
        col = i[1]
        grid[row][col].make_path()

def path_to_pxl(path):
    path_pxl = []
    for i in path:
        path_pxl.append([i[1]*(WIDTH/ROWS),i[0]*(WIDTH/ROWS),])

    return path_pxl

def pure_pursuit(path, current, lookout_dist):
    if not path:
        raise ValueError("Path is empty. Cannot calculate look-ahead point.")
    
    # Calculate distances from current position to each point in the path
    distances = [math.sqrt((point[0] - current[0])**2 + (point[1] - current[1])**2) for point in path]
    
    # Find the nearest point on the path
    min_value = min(distances)
    min_index = distances.index(min_value)
    
    # Determine the look-ahead index
    if min_value > lookout_dist:
        f = 0.3
        var_lookout_dist = math.ceil(lookout_dist + (min_value**f)) - 1
    else:
        var_lookout_dist = lookout_dist

    look_ahead_index = min_index + var_lookout_dist

    # Ensure the index is within bounds
    if look_ahead_index >= len(path):
        return path[-1]  # Return the last point if out of bounds

    return path[look_ahead_index]




    #points = [
    #(point1[0] + t * (point2[0] - point1[0]), point1[1] + t * (point2[1] - point1[1]))
    #    for t in [i / (num_points - 1) for i in range(num_points)] ]

    return 1


#____________________________________________________________________________________
# Functions

def ir_read(ir_list):
    irValues = []
    for i in range(len(ir_list)):
        irValues.append(ir_list[i].getValue())
    return irValues
    
def gps_to_pxl(gpsx,gpsy):
    X_o = abs(gpsx - GPS_ORIGIN[0]) # aligning origins
    Y_o = abs(gpsy - GPS_ORIGIN[1])

    gps_to_pxl_X = X_o * (WIDTH/ROOM_X) / 0.90436
    gps_to_pxl_Y = Y_o * (WIDTH/ROOM_X) / 0.90436

    return gps_to_pxl_X, gps_to_pxl_Y


def bearing():
    compX , compY , compZ = compass.getValues()
    theta = math.atan2(compX , compY) 
    bearing = math.degrees(theta)
    if bearing < 0.0:
        pass
        bearing = bearing + 360
    #bearing = 360 + bearing
    #print(bearing)
    return bearing

def translate(deg, omega, Speed=1, distance='inf'):
    rad = math.radians(deg+30-90)
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

def navigate(X1,Y1,X2,Y2):
    try:
        delta_x = X2 - X1
        delta_y = Y2 - Y1
        distance_to_target = math.sqrt(delta_x**2 + delta_y**2)
        theta =  math.atan2(delta_y, delta_x)  # atan2 automatically handles signs
        if theta < 0.0:
            theta = theta + (2 * math.pi)
            pass
        degree = math.degrees(theta)
        degree =  degree  - bearing() - 270
        if distance_to_target < distance_threshold:
            speed = min(translation_speed, distance_to_target /1.5)  # Scaled speed
        else: speed = translation_speed
        
        translate(degree,0,speed)
        
        #print(degree)
    except ZeroDivisionError:
        pass

#____________________________________________________________________________________
# MAIN loop:

while robot.step(TIME_STEP) != -1:
    grid_loaded = False
    run = False
    calc_path = True
    print("Robotino Controller\n==================\n")
    print("[1] Load Grid File")
    print("[2] Random Walk")
    print("[3] Make New Grid")
    print("[Save] Save Current Grid")
    print('[R] Run')

    while not run and robot.step(TIME_STEP) != -1: # Menu Selection Loop
        key=keyboard.getKey()
        #if (key==ord('F')):

        #option = input("Enter Option No. : ")
        if key==ord('1'):
            grid = load_grid_file(grid_file_address)
            #print(grid)
            grid_loaded = True
            sleep(0.5)
        elif key==ord('2'):
            sleep(0.5)
        elif key==ord('3'):
            WIN = pygame.display.set_mode((WIDTH, WIDTH))
            grid = edit_grid_window(WIN,WIDTH)
            grid_loaded = True
            sleep(0.5)
        elif key==Keyboard.CONTROL+ord('S'):
            save_grid_file(grid,grid_file_address)
            sleep(0.5)

        elif key==ord('R') and grid_loaded:
            run = True
            sleep(0.5)  

    while run and robot.step(TIME_STEP) != -1: # Run Loop, the Navigation starts here.
        
        if calc_path == True:
            path = algorithm(grid, start_node(grid), end_node(grid))
            WIN = pygame.display.set_mode((WIDTH, WIDTH))
            show_grid_window(WIN, grid, WIDTH)
            calc_path = False


            
        gpsX , gpsY , gpsZ = gps.getValues()
        gpsX = round(gpsX * 100000 , 8)
        gpsY = round(gpsY * 100000 , 8)
        #print("       ",gpsX, " , ",gpsY)
        gpsX_pxl,gpsY_pxl = gps_to_pxl(gpsX,gpsY)

        LA_X , LA_Y = pure_pursuit(path_to_pxl(path), [gpsX_pxl, gpsY_pxl], 2)
        print(round(gpsX_pxl,1),round(gpsY_pxl,1), "  |  " ,LA_X , LA_Y )
        navigate(gpsX_pxl, gpsY_pxl,LA_X , LA_Y )
        #print(path_to_pxl(path))




print("DONE")



"""
while robot.step(TIME_STEP) != -1: # Runaway loop
    
    key=keyboard.getKey()
    if (key==ord('F')):
         print('Ctrl+B is pressed')
    
    #print(ir_read(ir))
    pass
    
    #print(bearing())
    gpsX , gpsY , gpsZ = gps.getValues()
    gpsX = round(gpsX * 100000 , 8)
    gpsY = round(gpsY * 100000 , 8)
    print("       ",gpsX, " , ",gpsY)
    navigate(gpsX,gpsY,1,1)
""" 






# Enter here Tests.
# _________________
"""
#Test: to check grid object
grid = make_grid(10,100)
x = grid[0][9].y
save_grid_file(grid)
print(x)

#Test: Dumping grid object contents to a file
grid = make_grid(10,100)
x = grid[0][2].y
print(save_grid_file(grid,grid_file_address))
y = load_grid_file(grid_file_address)
print(x,y[0][2].y)



"""
