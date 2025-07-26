"""
Depth-First Search (DFS) Maze Explorer with Map and Metrics

This controller implements a stack-based DFS algorithm using proximity sensors
to explore the maze. It records the robot's path, logs visited cells, and saves
metrics for comparison with wall-following and random approaches.

Key metrics captured:
- Total number of control steps (iterations)
- Number of unique maze cells visited
- Total run time from START to END

Purpose:
This controller allows structured maze traversal with backtracking. It prioritises
exploration depth and provides a useful contrast to reactive algorithms.
"""

from controller import Supervisor
import math

# === CONSTANTS ===
MAX_SPEED = 6.28
PROX_THRESHOLD = 80
MAZE_SIZE = 12
ARENA_SIZE = 3.0
CELL_SIZE = ARENA_SIZE / MAZE_SIZE
STOP_RADIUS = 0.1
WHEEL_RADIUS = 0.0205

START_CELL = (0, 6)
END_CELL = (11, 6)

HEADINGS = ['N', 'E', 'S', 'W']
DIR_VECTORS = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
DIR_SENSORS = {
    'N': [0, 7],
    'E': [1],
    'S': [4],
    'W': [5]
}

def cellToWorld(row, col):
    x = -1.5 + (col + 0.5) * CELL_SIZE
    y = -1.5 + (row + 0.5) * CELL_SIZE
    return x, y, 0.01

def worldToCell(x, y):
    col = int((x + 1.5) / CELL_SIZE)
    row = int((y + 1.5) / CELL_SIZE)
    return row, col

def dropMarker(supervisor, pos, radius=0.01, color=[0, 0, 1]):
    children = supervisor.getRoot().getField('children')
    sphere = (
        f'Transform {{ translation {pos[0]} {pos[1]} {pos[2]} '
        'children [ Shape { appearance Appearance { material Material { diffuseColor ' +
        f'{color[0]} {color[1]} {color[2]}' +
        ' } } geometry Sphere { radius ' + f'{radius}' + ' } } ] }}'
    )
    children.importMFNodeFromString(-1, sphere)

def saveMap(matrix, steps, visited, duration, file="dfs_map.txt"):
    with open(file, 'w') as f:
        f.write("# DFS Maze Explorer Output\n")
        f.write(f"Total steps: {steps}\n")
        f.write(f"Cells visited: {visited}\n")
        f.write(f"Duration: {duration:.2f} seconds\n\n")
        for row in matrix:
            f.write(" ".join(row) + "\n")

def rotateTo(robot, leftMotor, rightMotor, current, target):
    idxC = HEADINGS.index(current)
    idxT = HEADINGS.index(target)
    delta = (idxT - idxC) % 4
    duration = 18

    if delta == 0:
        return current
    elif delta == 1:
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        robot.step(duration)
    elif delta == 2:
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        robot.step(duration * 2)
    elif delta == 3:
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(-MAX_SPEED)
        robot.step(duration)
    return target

def moveOneCell(robot, leftMotor, rightMotor, timestep):
    duration = int((CELL_SIZE / (WHEEL_RADIUS * MAX_SPEED)) * 1000 * 0.95)
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    robot.step(duration)
    for _ in range(3):
        robot.step(timestep)

def checkClear(psValues, direction):
    buffer = 10
    return all(psValues[i] < (PROX_THRESHOLD - buffer) for i in DIR_SENSORS[direction])

def runDFS(robot):
    timestep = int(robot.getBasicTimeStep())
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    prox = [robot.getDevice(f'ps{i}') for i in range(8)]
    for p in prox:
        p.enable(timestep)

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    mazeMap = [['?' for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    visited = set()
    stack = [START_CELL]
    heading = 'N'
    current = START_CELL
    steps = 0
    goalReached = False
    startTime = robot.getTime()

    print(f"Starting DFS from {START_CELL} to {END_CELL}")

    while robot.step(timestep) != -1:
        psValues = [p.getValue() for p in prox]
        x, y, _ = gps.getValues()
        row, col = worldToCell(x, y)
        current = (row, col)

        if current not in visited:
            visited.add(current)
            mazeMap[row][col] = '0'
            dropMarker(robot, cellToWorld(row, col))
            print(f"Visited cell {current}")

        if current == END_CELL:
            print("Reached goal cell!")
            goalReached = True
            break

        moved = False
        for dir in HEADINGS:
            dr, dc = DIR_VECTORS[dir]
            nr, nc = row + dr, col + dc
            nextCell = (nr, nc)
            if (0 <= nr < MAZE_SIZE and 0 <= nc < MAZE_SIZE and
                nextCell not in visited and
                checkClear(psValues, dir)):

                stack.append(current)
                heading = rotateTo(robot, leftMotor, rightMotor, heading, dir)
                moveOneCell(robot, leftMotor, rightMotor, timestep)

                x, y, _ = gps.getValues()
                newRow, newCol = worldToCell(x, y)
                newCell = (newRow, newCol)

                if newCell == current:
                    print(f"Blocked! Tried to move {dir} but stayed in {current}")
                    visited.add(nextCell)
                    continue
                else:
                    current = newCell
                    steps += 1
                    moved = True
                    break

        if not moved:
            print(f"No move possible from {current}, backtracking...")
            if stack:
                prev = stack.pop()
                drow, dcol = prev[0] - row, prev[1] - col
                for h, (dr, dc) in DIR_VECTORS.items():
                    if (dr, dc) == (drow, dcol):
                        heading = rotateTo(robot, leftMotor, rightMotor, heading, h)
                        moveOneCell(robot, leftMotor, rightMotor, timestep)
                        current = prev
                        steps += 1
                        break
            else:
                print("DFS complete — no path to goal.")
                break

    duration = robot.getTime() - startTime
    if goalReached:
        print(f"\nGoal reached in {steps} steps, {len(visited)} cells, {duration:.2f}s")
    else:
        print(f"\nDFS terminated. Goal not reached. Steps: {steps}, Cells visited: {len(visited)}, Duration: {duration:.2f}s")

    saveMap(mazeMap, steps, len(visited), duration)

if __name__ == "__main__":
    robot = Supervisor()
    runDFS(robot)
