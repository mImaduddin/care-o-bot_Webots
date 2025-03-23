
from scipy.ndimage import distance_transform_edt
import numpy as np
import matplotlib.pyplot as plt

def compute_cost_map_efficient(input_grid):

    occupancy_grid = np.array(input_grid)
    cost_map = distance_transform_edt(occupancy_grid == 0)

    # Normalize the 

    max_cost = np.max(cost_map)
    #cost_map[cost_map == 1] = 0
    if max_cost > 0:
        cost_map = cost_map /  max_cost

    #cost_map = 1 - cost_map
 
    #cost_map[cost_map == 1] = 0

    return cost_map.tolist()

occupancy_grid_random_wall = np.zeros((100, 100))

occupancy_grid_random_wall[0, :] = 1 
occupancy_grid_random_wall[-1, :] = 1 
occupancy_grid_random_wall[:, 0] = 1  
occupancy_grid_random_wall[:, -1] = 1  

np.random.seed(42)  # For reproducibility
x_start, y_start = np.random.randint(20, 60), np.random.randint(20, 60)
x_size, y_size = np.random.randint(5, 15), np.random.randint(5, 15)

occupancy_grid_random_wall[x_start:x_start+x_size, y_start:y_start+y_size] = 1

cost_map_random_wall_efficient = compute_cost_map_efficient(occupancy_grid_random_wall)
print(cost_map_random_wall_efficient)
    
plt.imshow(cost_map_random_wall_efficient, cmap='hot', interpolation='nearest')
plt.colorbar(label='Normalized Cost')
plt.title('Normalized Cost Map (Euclidean Distance Transform)')
plt.show()
