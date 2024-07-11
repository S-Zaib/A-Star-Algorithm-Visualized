import random
import math
import heapq
import time
import tkinter as tk
from tkinter import ttk
import numpy as np

# Environment Setup
GRID_SIZE = 15
CELL_SIZE = 40
grid = np.zeros((GRID_SIZE, GRID_SIZE))

# Function to create clusters of obstacles
def create_cluster(center, size, grid, value):
    cx, cy = center
    for i in range(cx - size, cx + size + 1):
        for j in range(cy - size, cy + size + 1):
            if 0 <= i < GRID_SIZE and 0 <= j < GRID_SIZE:
                grid[i, j] = value

# Add clusters of buildings, houses, and vehicles
building_centers = [(3, 3), (3, 10), (11, 8)]
house_centers = [(10, 2)]
vehicle_locations = [(1, 1), (1, 13), (13, 1), (13, 13), (7, 1), (1, 7), (13, 7), (7, 13)]

for center in building_centers:
    create_cluster(center, 2, grid, 1) # 1 represents an obstacle

for center in house_centers:
    create_cluster(center, 1, grid, 1) # 1 represents an obstacle

for location in vehicle_locations:
    grid[location] = 2  # 2 represents a vehicle

# Set initial start location
start_location = (0, 0)

# Generate 5 random delivery locations
delivery_locations = []
while len(delivery_locations) < 5:
    location = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
    if grid[location] == 0 and location != start_location and location not in delivery_locations:
        delivery_locations.append(location)

print("Delivery locations:", delivery_locations)

# Algorithm Implementation
def euclidean_distance(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def get_neighbors(current, grid):
    x, y = current
    neighbors = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # 4-directional movement (no diagonals)
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[nx, ny] == 0:
            neighbors.append((nx, ny))
    return neighbors

def best_first_search(start, goal, grid):
    frontier = [(euclidean_distance(start, goal), start)]
    came_from = {start: None}
    
    while frontier:
        current = heapq.heappop(frontier)[1]
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        for neighbor in get_neighbors(current, grid):
            if neighbor not in came_from:
                priority = euclidean_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    
    return None

def a_star(start, goal, grid):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    while frontier:
        current = heapq.heappop(frontier)[1]
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        for neighbor in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + 1  # Consistent cost for each move
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + euclidean_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    
    return None

# Tkinter GUI Setup
root = tk.Tk()
root.title("Delivery Robot Simulation")

canvas = tk.Canvas(root, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE)
canvas.pack(side=tk.LEFT)

# Draw grid
for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        canvas.create_rectangle(i*CELL_SIZE, j*CELL_SIZE, (i+1)*CELL_SIZE, (j+1)*CELL_SIZE, fill="white", outline="gray")

# Draw obstacles
for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        if grid[i, j] == 1:
            canvas.create_rectangle(i*CELL_SIZE, j*CELL_SIZE, 
                                    (i+1)*CELL_SIZE, (j+1)*CELL_SIZE, fill="red")
        elif grid[i, j] == 2:
            canvas.create_rectangle(i*CELL_SIZE, j*CELL_SIZE, 
                                    (i+1)*CELL_SIZE, (j+1)*CELL_SIZE, fill="gray")

# Draw delivery locations
goal_dots = {}
for location in delivery_locations:
    goal_dots[location] = canvas.create_oval(location[0]*CELL_SIZE, location[1]*CELL_SIZE, 
                                             (location[0]+1)*CELL_SIZE, (location[1]+1)*CELL_SIZE, fill="green")

# Robot representation with smaller size
robot_radius = 10  # Smaller radius for the robot
robot = canvas.create_oval(start_location[0]*CELL_SIZE + robot_radius, start_location[1]*CELL_SIZE + robot_radius, 
                           (start_location[0]+1)*CELL_SIZE - robot_radius, (start_location[1]+1)*CELL_SIZE - robot_radius, fill="blue")

# Legend Frame
legend_frame = tk.Frame(root)
legend_frame.pack(side=tk.RIGHT, padx=10)

legend_title = tk.Label(legend_frame, text="Legend", font=("Arial", 14))
legend_title.pack()

legend_items = [
    ("Robot", "blue"),
    ("Obstacle", "red"),
    ("Vehicle", "gray"),
    ("Delivery Location/Goal", "green"),
    ("Goal Reached", "yellow"),
    ("New Obstacle", "orange")
]

for item, color in legend_items:
    legend_color = tk.Canvas(legend_frame, width=20, height=20, bg=color)
    legend_color.pack(side=tk.TOP, pady=2)
    legend_label = tk.Label(legend_frame, text=item)
    legend_label.pack(side=tk.TOP, pady=2)


def move_robot(x, y, goal):
    canvas.coords(robot, x*CELL_SIZE + robot_radius, y*CELL_SIZE + robot_radius, 
                  (x+1)*CELL_SIZE - robot_radius, (y+1)*CELL_SIZE - robot_radius)
    if (x, y) == goal:
        canvas.itemconfig(goal_dots[goal], fill="yellow")  # Change color of the goal dot when reached
    root.update()

def add_new_obstacle():
    while True:
        new_obstacle = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        if grid[new_obstacle] == 0 and new_obstacle not in delivery_locations:
            grid[new_obstacle] = 1
            canvas.create_rectangle(new_obstacle[0]*CELL_SIZE, new_obstacle[1]*CELL_SIZE, 
                                    (new_obstacle[0]+1)*CELL_SIZE, (new_obstacle[1]+1)*CELL_SIZE, fill="orange")
            print(f"New obstacle added at {new_obstacle}")
            return new_obstacle

def execute_delivery():
    current_location = start_location
    total_time_bfs = 0
    total_time_astar = 0
    total_path_length_bfs = 0
    total_path_length_astar = 0
    
    for i, goal in enumerate(delivery_locations):
        print(f"\nDelivery {i+1}: From {current_location} to {goal}")
        
        # Best-First Search
        start_time = time.time()
        path_bfs = best_first_search(current_location, goal, grid)
        end_time = time.time()
        total_time_bfs += end_time - start_time
        
        # A* Search
        start_time = time.time()
        path_astar = a_star(current_location, goal, grid)
        end_time = time.time()
        total_time_astar += end_time - start_time
        
        if path_bfs and path_astar:
            print(f"Best-First Search Path: {path_bfs}")
            print(f"A* Search Path: {path_astar}")
            
            # Use A* path for visualization
            for step in path_astar:
                move_robot(step[0], step[1], goal)
                time.sleep(0.3)
            
            total_path_length_bfs += len(path_bfs) - 1
            total_path_length_astar += len(path_astar) - 1
            current_location = goal
            
            # Add a new obstacle after each successful delivery
            add_new_obstacle()
            
            # Change goal state for next delivery
            if i < len(delivery_locations) - 1:
                delivery_locations[i+1] = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
                while grid[delivery_locations[i+1]] != 0 or delivery_locations[i+1] in delivery_locations[:i+1]:
                    delivery_locations[i+1] = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
                goal_dots[delivery_locations[i+1]] = canvas.create_oval(delivery_locations[i+1][0]*CELL_SIZE, delivery_locations[i+1][1]*CELL_SIZE, 
                                                                       (delivery_locations[i+1][0]+1)*CELL_SIZE, (delivery_locations[i+1][1]+1)*CELL_SIZE, fill="green")
                print(f"New goal location: {delivery_locations[i+1]}")
        else:
            print(f"No path found to {goal}")
    
    # Performance evaluation
    print(f"\nPerformance Evaluation:")
    print(f"Best-First Search:")
    print(f"  Total path length: {total_path_length_bfs} steps")
    print(f"  Total execution time: {total_time_bfs:.6f} seconds")
    print(f"  Average path length: {total_path_length_bfs/len(delivery_locations):.2f} steps")
    print(f"A* Search:")
    print(f"  Total path length: {total_path_length_astar} steps")
    print(f"  Total execution time: {total_time_astar:.6f} seconds")
    print(f"  Average path length: {total_path_length_astar/len(delivery_locations):.2f} steps")

# Start button
start_button = ttk.Button(root, text="Start Delivery", command=execute_delivery)
start_button.pack()

root.mainloop()
