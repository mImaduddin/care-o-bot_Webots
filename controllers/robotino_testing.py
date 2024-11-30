"""Robotino_careobot controller."""

#from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Compass
import math
import pickle
import pygame
import numpy as np
from simple_pid import PID
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt
from time import sleep


WHEEL_RADIUS = 0.063 #meter
ROBOT_RADIUS = 0.2266 #meter
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
IR_ANGLES = [90.0, 130.0, 170.0, 210.0, 250.0, 290.0, 330.0, 10.0, 40.0]
IR_12oclock = 0.63 #meters max dist


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


"""
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# initialize devices
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# IR Sensors: ir1(12oclock), ir2(11oclock), ... , ir9(1oclock) (clock from top view)
# IR Sensor: 0.31 when nothing , 3.0 when touching
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
"""


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

    def is_open(self):
        return self.color == GREENS

    def is_barrier(self):
        if self.color == BLACK:
            return 1
        return 0

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
    while run: #and robot.step(TIME_STEP) != -1:
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
    while run:# and robot.step(TIME_STEP) != -1:
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

def compute_cost_map(input_grid):
    
    # Compute the Euclidean distance transform. The distance transform computes the distance from each
    # free cell (0) to the nearest occupied cell (1).
    occupancy_grid = np.array(input_grid)
    cost_map = distance_transform_edt(occupancy_grid == 0)

    # Normalize the cost map
    max_cost = np.max(cost_map)
    cost_map[cost_map == 1] = 0
    if max_cost > 0:
        cost_map = cost_map /  max_cost

    cost_map = 1 - cost_map
 
    cost_map[cost_map == 1] = 0
    return cost_map.tolist()

def algorithm(grid, start, end):
    grid_matrix = [[row[i].is_barrier() for row in grid] for i in range(len(grid[0]))] # convert grid to boolean or numeric matrix

    #Making Boundary wall (TEMPORARY)
    for i in range(len(grid_matrix)):
        grid_matrix[0][i] = 1  # Top row
        grid_matrix[-1][i] = 1  # Bottom row
        grid_matrix[i][0] = 1  # Left column
        grid_matrix[i][-1] = 1  # Right column

    cost_map_matrix = compute_cost_map(grid_matrix)
    plt.imshow(cost_map_matrix, cmap='hot', interpolation='nearest')
    #print(cost_map_matrix)
    grid = Grid(matrix=cost_map_matrix)
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


    
def gps_to_pxl(gpsx,gpsy):
    X_o = abs(gpsx - GPS_ORIGIN[0]) # aligning origins
    Y_o = abs(gpsy - GPS_ORIGIN[1])

    gps_to_pxl_X = X_o * (WIDTH/ROOM_X) / 0.90436
    gps_to_pxl_Y = Y_o * (WIDTH/ROOM_X) / 0.90436

    return gps_to_pxl_X, gps_to_pxl_Y

def meter_to_pxl(value):
    return (value*(WIDTH/ROOM_X))
    IR

IR_position_in_pxl = meter_to_pxl(ROBOT_RADIUS)
IR_12oclock_in_pxl = meter_to_pxl(IR_12oclock)
IR_to_pxl_factor = (IR_12oclock_in_pxl - IR_position_in_pxl) / (3.0 - 0.3)


def ir_read_pxl(ir_list):
    irValues = []
    for i in range(len(ir_list)):
        #irValues.append((IR_12oclock_in_pxl - ( (IR_12oclock_in_pxl - IR_position_in_pxl) * ( ir_list[i].getValue() - 0.3) / (3 - 0.3) )))
        irValues.append((IR_12oclock_in_pxl - ( (IR_12oclock_in_pxl - IR_position_in_pxl) * ( ir_list[i] - 0.3) / (3 - 0.3) )))
    return irValues

def bearing():
    #compX , compY , compZ = compass.getValues()
    compX , compY , compZ = compass
    theta = math.atan2(compX , compY) 
    bearing = math.degrees(theta)
    if bearing < 0.0:
        pass
        bearing = bearing + 360
    #bearing = 360 + bearing
    #print(bearing)
    return bearing

def calculate_weighted_centroid(polygon, weights):
    """
    Calculate the weighted centroid of a polygon.
    
    Parameters:
        polygon (list of tuple): List of (x, y) coordinates representing the vertices of the polygon.
        weights (list of float): List of weights corresponding to each vertex, where a higher weight indicates more importance (e.g., farther from an obstacle).
    
    Returns:
        tuple: (Cx, Cy) - coordinates of the weighted centroid.
    """
    n = len(polygon)
    if n < 3:
        raise ValueError("A polygon must have at least 3 vertices.")
    if len(weights) != n:
        raise ValueError("The number of weights must match the number of vertices.")
    
    # Initialize weighted sums for centroid coordinates
    weighted_area = 0
    Cx = 0
    Cy = 0

    # Loop through vertices and calculate weighted centroid
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]  # Wrap to first vertex for the last edge
        cross = x1 * y2 - x2 * y1  # Cross product
        weighted_area += cross
        Cx += (x1 + x2) * cross * weights[i]
        Cy += (y1 + y2) * cross * weights[i]

    # Finalize calculations
    weighted_area *= 0.5
    if weighted_area == 0:
        raise ValueError("Polygon area is zero, check if vertices are in the correct order and non-degenerate.")
    
    Cx /= (6 * weighted_area)
    Cy /= (6 * weighted_area)
    
    return Cx, Cy

"""
# Example usage:
polygon = [(0, 0), (4, 0), (4, 3), (0, 3)]  # Rectangle
weights = [1, 1, 1, 1]  # Equal weights (no special weighting)
centroid = calculate_weighted_centroid(polygon, weights)
print("Weighted Centroid:", centroid)

# With varying weights (e.g., emphasizing points farther from obstacles):
weights = [1, 2, 3, 4]  # Increasing weight from left to right
centroid = calculate_weighted_centroid(polygon, weights)
print("Weighted Centroid with Varying Weights:", centroid)
"""

def calculate_centroid(polygon):
    area = 0
    Cx = 0
    Cy = 0
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]  # Wrap to first vertex for the last edge
        cross = x1 * y2 - x2 * y1  # Cross product
        area += cross
        Cx += (x1 + x2) * cross
        Cy += (y1 + y2) * cross

    area *= 0.5
    
    Cx /= (6 * area)
    Cy /= (6 * area)
    
    return Cx, Cy


def obstacle_polygon(ir_values_in_pxl, angles, gps_pxl, bearing_deg):
    # values invertor
    polygon = []
    j = 0
    for i in ir_values_in_pxl:
        polygon.append([(gps_pxl[0] + i * math.cos(math.radians(bearing_deg + angles[j]))) , (gps_pxl[1] + i * math.sin(math.radians(bearing_deg + angles[j]))) ])      
        j = j + 1
    return polygon


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




def obstacle_avoid_point(gps,direction):
    ir_values = ir_read_pxl(ir)
    polygon_list = obstacle_polygon(ir_values, IR_ANGLES, gps, 0)

    def line_intersection(p1, p2, p3 ,p4):
        # Unpack points from the input line segments
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        # Calculate the slopes of both lines
        # If the x-values of two points are the same, the line is vertical, so slope is infinite
        m1 = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
        m2 = (y4 - y3) / (x4 - x3) if x4 != x3 else float('inf')

        L = 1000   #pxl
        m = m2
        x4 , y4 = (x3 + ((L * m) / math.sqrt(1 + m**2))) , (y3 - (L / math.sqrt(1 + m**2)))
        x3 , y3 = (x3 + ((-L * m) / math.sqrt(1 + m**2))) , (y3 - (-L / math.sqrt(1 + m**2)))

        m2 = -1/m2 if m2 != 0 else float('inf')


        # Helper function to check if point (px, py) is on the segment (x1, y1) to (x2, y2)
        def on_segment(px, py, x1, y1, x2, y2):
            return min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2)

        # Check if lines are parallel
        if m1 == m2:
            return None  # Lines are parallel and do not intersect

        # Calculate the y-intercepts for both lines
        b1 = y1 - m1 * x1  # y-intercept of line 1
        b2 = y3 - m2 * x3  # y-intercept of line 2

        # Calculate the x-coordinate of the intersection
        x_intersect = (b2 - b1) / (m1 - m2)
        
        # Calculate the y-coordinate of the intersection by substituting x into one of the line equations
        y_intersect = m1 * x_intersect + b1

        # Check if the intersection point (x_intersect, y_intersect) is within both line segments
        if not (on_segment(x_intersect, y_intersect, x1, y1, x2, y2) and on_segment(x_intersect, y_intersect, x3, y3, x4, y4)):
            return None  # The intersection point is not within both line segments

        return x_intersect, y_intersect


    dx, dy = direction
    
    # Find the intersection points of the perpendicular line with the polygon edges
    intersection_points = []
    n = len(polygon_list)
    for i in range(n):
        p1 = polygon_list[i]
        p2 = polygon_list[(i + 1) % n]
        intersection = line_intersection( p1, p2, gps, (dx, dy))
        #print(intersection)
        if intersection:
            intersection_points.append(intersection)
    #print(intersection_points)


    x1, y1 = intersection_points[0]
    x2, y2 = intersection_points[1]
    
    # Calculate the cross product of the direction point with respect to the line
    direction_cross = (x2 - x1) * (dy - y1) - (y2 - y1) * (dx - x1)
    
    result_polygon = []
    for (xp, yp) in polygon_list:
        # Calculate cross product for current point
        point_cross = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
        
        # If cross products have the same sign, the point lies on the same side
        if (direction_cross > 0 and point_cross > 0) or (direction_cross < 0 and point_cross < 0):
            result_polygon.append((xp, yp))
    

    result_polygon.append(intersection_points[0])
    result_polygon.append(intersection_points[1])

    
    return result_polygon


    

def point_navigate(X1,Y1,X2,Y2):
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
        
        #print(degree)
    except ZeroDivisionError:
        pass
    return degree

def plot_polygon(points, screen_size=(800, 600), line_color=(0, 255, 0), background_color=(0, 0, 0)):
    """
    Plot a polygon using Pygame.
    
    Parameters:
        points (list): List of (x, y) tuples representing the vertices of the polygon.
        screen_size (tuple): Width and height of the Pygame window (default: (800, 600)).
        line_color (tuple): RGB color of the polygon lines (default: green).
        background_color (tuple): RGB color of the background (default: black).
    """
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption("Polygon Plotter")
    
    # Scale the points to fit within the screen size
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    
    def scale_point(x, y):
        """Scale points to fit within the Pygame screen."""
        scaled_x = int((x - min_x) / (max_x - min_x) * (screen_size[0] - 100)) + 50
        scaled_y = int((y - min_y) / (max_y - min_y) * (screen_size[1] - 100)) + 50
        return (scaled_x, scaled_y)

    # Scale all points
    scaled_points = [scale_point(x, y) for x, y in points]

    # Main loop
    running = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit() 
    
    # Clear the screen
    screen.fill(background_color)
    
    # Draw the polygon
    pygame.draw.polygon(screen, line_color, scaled_points, 1)  # Width = 1 for outline
    
    # Draw vertices as points
    for p in scaled_points:
        pygame.draw.circle(screen, (255, 0, 0), p, 5)  # Red dots for vertices

    # Update the display
    pygame.display.flip()

    







#____________________________________________________________________________________
# MAIN loop:

while True: # robot.step(TIME_STEP) != -1:
    grid_loaded = False
    run = False
    calc_path = True
    ir = [3,3,3,3,3,3,3,3,3]
    compass = [0,0,0]

    print("Robotino Controller\n==================\n")
    print("[1] Load Grid File")
    print("[2] Random Walk")
    print("[3] Make New Grid")
    print("[Save] Save Current Grid")
    print('[R] Run')

    while not run: # and robot.step(TIME_STEP) != -1: # Menu Selection Loop
        option = input("Enter Option No. : ")
        if option == "1":
            grid = load_grid_file(grid_file_address)
            grid_loaded = True
            pass
        elif option == "2":
            pass
        elif option == "3":
            WIN = pygame.display.set_mode((WIDTH, WIDTH))
            grid = edit_grid_window(WIN,WIDTH)
            grid_loaded = True
        elif option == "Save":
            save_grid_file(grid,grid_file_address)

        elif option == "R" and grid_loaded:
            run = True




    while run:# and robot.step(TIME_STEP) != -1: # Run Loop, the Navigation starts here.
        
        if calc_path == True:
            path = algorithm(grid, start_node(grid), end_node(grid))
            WIN = pygame.display.set_mode((WIDTH, WIDTH))
            show_grid_window(WIN, grid, WIDTH)
            calc_path = False


            
        gpsX , gpsY , gpsZ = (300,300,0)#gps.getValues()
        gpsX = round(gpsX * 100000 , 8)
        gpsY = round(gpsY * 100000 , 8)
        #print(gpsX, " , ",gpsY)
        gpsX_pxl, gpsY_pxl = gps_to_pxl(gpsX,gpsY)
        LA_X , LA_Y = pure_pursuit(path_to_pxl(path), [gpsX_pxl, gpsY_pxl], 2)
        
        #print(obstacle_avoid_point((gpsX_pxl, gpsY_pxl ),(300 , 300 )))
        #print(obstacle_polygon(ir_read_pxl(ir), IR_ANGLES, [gpsX_pxl, gpsY_pxl], bearing()-90))
        #print(ir_read_pxl(ir))
        #print(bearing())
        #print(round(gpsX_pxl,1),round(gpsY_pxl,1), "  |  " ,LA_X , LA_Y )
        
        #plot_polygon(obstacle_polygon(ir_read_pxl(ir), IR_ANGLES, [gpsX_pxl, gpsY_pxl], bearing()-90))

        cx , cy = calculate_centroid(obstacle_avoid_point((gpsX_pxl,gpsY_pxl) , (LA_X,LA_Y)))
        #cx , cy = calculate_centroid(obstacle_polygon(ir_read_pxl(ir), IR_ANGLES, [gpsX_pxl, gpsY_pxl], bearing()+90))
        degree = point_navigate(gpsX_pxl, gpsY_pxl, cx, cy )
        #translate(degree,0,5)


        #sleep(1)

        #translate(degree,0,speed)
        #print(path_to_pxl(path))




print("DONE")

