"""a_star_maze_solver controller for e-puck robot."""

from controller import Robot, Motor, DistanceSensor
import math
import heapq

# E-puck device names
DIST_SENSOR_NAMES = [
    'ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7'
]
LEFT_MOTOR = 'left wheel motor'
RIGHT_MOTOR = 'right wheel motor'

# Maze/grid parameters (example: 10x10 grid)
MAZE_SIZE = 10
CELL_SIZE = 0.05  # 5cm per cell (adjust as per your maze)

# Directions: up, right, down, left
DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]

# Create the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice(LEFT_MOTOR)
right_motor = robot.getDevice(RIGHT_MOTOR)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize distance sensors
sensors = []
for name in DIST_SENSOR_NAMES:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

# Maze map: 0 = free, 1 = wall, -1 = unknown
maze_map = [[-1 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
maze_map[0][0] = 0  # Mark starting cell as free

# Robot state
robot_pos = [0, 0]  # Start at (0,0)
robot_dir = 0  # 0: up, 1: right, 2: down, 3: left
explored = False
path = []
goal = (MAZE_SIZE-1, MAZE_SIZE-1)  # Example: bottom-right corner

# Movement parameters
MAX_SPEED = 6.28
TURN_SPEED = 0.5 * MAX_SPEED
FORWARD_SPEED = 0.8 * MAX_SPEED
SENSOR_THRESHOLD = 80.0  # Adjust based on sensor calibration

# Helper functions
def in_bounds(x, y):
    return 0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE

def left_dir(d):
    return (d - 1) % 4

def right_dir(d):
    return (d + 1) % 4

def update_map_with_sensors():
    """Update the maze map based on current sensor readings and robot position/direction."""
    global maze_map
    x, y = robot_pos
    d = robot_dir
    # Front sensor (ps7, ps0), left (ps5, ps6), right (ps1, ps2)
    # Map: 0=up, 1=right, 2=down, 3=left
    # Check front
    front_dx, front_dy = DIRS[d]
    if in_bounds(x + front_dx, y + front_dy):
        front_val = max(sensors[7].getValue(), sensors[0].getValue())
        if front_val > SENSOR_THRESHOLD:
            maze_map[x + front_dx][y + front_dy] = 1  # wall
        else:
            if maze_map[x + front_dx][y + front_dy] == -1:
                maze_map[x + front_dx][y + front_dy] = 0  # free
    # Check left
    left_d = left_dir(d)
    left_dx, left_dy = DIRS[left_d]
    if in_bounds(x + left_dx, y + left_dy):
        left_val = max(sensors[5].getValue(), sensors[6].getValue())
        if left_val > SENSOR_THRESHOLD:
            maze_map[x + left_dx][y + left_dy] = 1
        else:
            if maze_map[x + left_dx][y + left_dy] == -1:
                maze_map[x + left_dx][y + left_dy] = 0
    # Check right
    right_d = right_dir(d)
    right_dx, right_dy = DIRS[right_d]
    if in_bounds(x + right_dx, y + right_dy):
        right_val = max(sensors[1].getValue(), sensors[2].getValue())
        if right_val > SENSOR_THRESHOLD:
            maze_map[x + right_dx][y + right_dy] = 1
        else:
            if maze_map[x + right_dx][y + right_dy] == -1:
                maze_map[x + right_dx][y + right_dy] = 0
    # Mark current cell as free
    maze_map[x][y] = 0

def move_forward():
    left_motor.setVelocity(FORWARD_SPEED)
    right_motor.setVelocity(FORWARD_SPEED)

def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_left():
    left_motor.setVelocity(-TURN_SPEED)
    right_motor.setVelocity(TURN_SPEED)

def turn_right():
    left_motor.setVelocity(TURN_SPEED)
    right_motor.setVelocity(-TURN_SPEED)

def a_star(start, goal, maze):
    """A* pathfinding on a grid maze."""
    heap = []
    heapq.heappush(heap, (0 + heuristic(start, goal), 0, start, [start]))
    visited = set()
    while heap:
        f, cost, current, path = heapq.heappop(heap)
        if current == goal:
            return path
        if current in visited:
            continue
        visited.add(current)
        x, y = current
        for i, (dx, dy) in enumerate(DIRS):
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny) and maze[nx][ny] == 0 and (nx, ny) not in visited:
                heapq.heappush(heap, (cost + 1 + heuristic((nx, ny), goal), cost + 1, (nx, ny), path + [(nx, ny)]))
    return []

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def explore_and_map():
    """Simple left-wall-following exploration to build the map."""
    global robot_pos, robot_dir, explored
    update_map_with_sensors()
    x, y = robot_pos
    d = robot_dir
    # Try left
    left_d = left_dir(d)
    left_dx, left_dy = DIRS[left_d]
    if in_bounds(x + left_dx, y + left_dy) and maze_map[x + left_dx][y + left_dy] in (0, -1):
        # Turn left and move
        turn_left()
        robot_dir = left_d
    # Try forward
    elif in_bounds(x + DIRS[d][0], y + DIRS[d][1]) and maze_map[x + DIRS[d][0]][y + DIRS[d][1]] in (0, -1):
        move_forward()
    # Try right
    elif in_bounds(x + DIRS[right_dir(d)][0], y + DIRS[right_dir(d)][1]) and maze_map[x + DIRS[right_dir(d)][0]][y + DIRS[right_dir(d)][1]] in (0, -1):
        turn_right()
        robot_dir = right_dir(d)
    else:
        # Dead end, turn around
        turn_left()
        turn_left()
        robot_dir = (d + 2) % 4
    # Simulate movement (for demo, in real robot use odometry)
    # Here, just update position after a few steps
    # In real code, use encoders/odometry to update robot_pos
    # For now, assume one cell per move
    robot_pos = [x + DIRS[robot_dir][0], y + DIRS[robot_dir][1]]
    if robot_pos == [goal[0], goal[1]]:
        explored = True
        stop()

def follow_path(path):
    """Follow the path by moving cell to cell."""
    global robot_pos, robot_dir
    for next_cell in path[1:]:
        x, y = robot_pos
        nx, ny = next_cell
        # Determine direction to next cell
        for i, (dx, dy) in enumerate(DIRS):
            if x + dx == nx and y + dy == ny:
                target_dir = i
                break
        # Turn to face target_dir
        while robot_dir != target_dir:
            turn_right()
            robot_dir = right_dir(robot_dir)
        # Move forward one cell
        move_forward()
        # Simulate movement (in real robot use odometry)
        robot_pos = [nx, ny]
    stop()

# Main loop
while robot.step(timestep) != -1:
    if not explored:
        explore_and_map()
        if explored:
            stop()
            path = a_star(tuple(robot_pos), goal, maze_map)
            if path:
                print("Path found:", path)
    elif path:
        follow_path(path)
        path = []
    else:
        stop()
