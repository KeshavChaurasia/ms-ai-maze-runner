"""
Right-Hand Wall-Following Maze Explorer with Map and Metrics

This variant of the wall-following algorithm uses a **right-hand rule** to navigate the maze.
It tracks and logs the same key metrics as the left-hand strategy:

    - Total number of control steps (iterations)
    - Number of unique maze cells visited
    - Total run time from START to END

Purpose:
This controller is intended as a comparative experiment to the original **left-hand**
wall follower. By changing only the direction of preference (right wall vs left wall),
we can assess how maze asymmetry impacts pathfinding efficiency and overall performance.

All outputs (including the matrix map and statistics) follow the same format for easy
side-by-side comparison.
"""

from controller import Supervisor
import math

# === CONSTANTS ===
MAX_SPEED = 6.28
PROX_THRESHOLD = 80
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.053

MAZE_SIZE = 12
ARENA_SIZE = 3.0
CELL_SIZE = ARENA_SIZE / MAZE_SIZE
STOP_RADIUS = 0.1

START_CELL = (0, 6)
END_CELL = (11, 6)

# === POSITION CONVERSION ===
def cellToWorld(row, col):
    x = -1.5 + (col + 0.5) * CELL_SIZE
    y = -1.5 + (row + 0.5) * CELL_SIZE
    z = 0.0
    return x, y, z

def worldToCell(x, y):
    col = int((x + 1.5) / CELL_SIZE)
    row = int((y + 1.5) / CELL_SIZE)
    return row, col

# === MARKER DROPPING ===
def dropMarker(supervisor, position, radius=0.01, color=[0, 0, 1]):
    """Drop a coloured marker sphere at the given position in Webots."""
    childrenField = supervisor.getRoot().getField('children')
    protoStr = (
        f'Transform {{ translation {position[0]} {position[1]} {position[2]} '
        'children [ '
        'Shape { '
        'appearance Appearance { material Material { diffuseColor ' +
        f'{color[0]} {color[1]} {color[2]}' +
        ' } } '
        f'geometry Sphere {{ radius {radius} }} '
        '} ] }'
    )
    childrenField.importMFNodeFromString(-1, protoStr)

# === MAP OUTPUT ===
def saveMap(matrix, stepCount, visitedCount, durationSeconds, filename="right_wall_map.txt"):
    """Write the explored maze matrix and metrics to a file."""
    with open(filename, "w") as f:
        f.write("# Right-hand wall follower run summary\n")
        f.write(f"Total steps: {stepCount}\n")
        f.write(f"Cells visited: {visitedCount}\n")
        f.write(f"Run time: {durationSeconds:.2f} seconds\n\n")
        f.write("Maze map:\n")
        for row in matrix:
            f.write(" ".join(row) + "\n")

# === MAIN CONTROLLER ===
def runRightWallFollower(robot):
    """Main control loop for right-hand wall-following navigation."""
    timestep = int(robot.getBasicTimeStep())

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    proxSensors = [robot.getDevice(f'ps{i}') for i in range(8)]
    for sensor in proxSensors:
        sensor.enable(timestep)

    # Map setup
    mazeMap = [['?' for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    visitedCells = set()
    stepCount = 0
    startTime = robot.getTime()

    print(f"Start: {START_CELL} -> {cellToWorld(*START_CELL)}")
    print(f"End:   {END_CELL} -> {cellToWorld(*END_CELL)}")

    while robot.step(timestep) != -1:
        x, y, z = gps.getValues()
        row, col = worldToCell(x, y)

        if 0 <= row < MAZE_SIZE and 0 <= col < MAZE_SIZE:
            mazeMap[row][col] = '0'
            visitedCells.add((row, col))

        if stepCount % 10 == 0:
            dropMarker(robot, (x, y, z + 0.01), color=[0, 0, 1])  # Blue marker

        stepCount += 1

        # Sensor readings
        psValues = [sensor.getValue() for sensor in proxSensors]
        frontWall = psValues[7] > PROX_THRESHOLD or psValues[0] > PROX_THRESHOLD
        rightWall = psValues[2] > PROX_THRESHOLD  # Correct right-side sensor

        # Right-hand wall-following logic
        if frontWall:
            # Turn left to avoid obstacle
            leftSpeed = -MAX_SPEED / 2
            rightSpeed = MAX_SPEED
        elif rightWall:
            # Follow the wall on the right
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED
        else:
            # Turn right to find a wall
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED / 4

        # Goal check
        goalX, goalY, _ = cellToWorld(*END_CELL)
        distanceToGoal = math.hypot(x - goalX, y - goalY)
        print(f"Step {stepCount}: Cell ({row},{col}) | Distance to goal: {distanceToGoal:.2f}")

        if distanceToGoal < STOP_RADIUS:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            print(f"Goal reached at ({x:.2f}, {y:.2f}).")
            break

        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

    # Completion logging
    endTime = robot.getTime()
    durationSeconds = endTime - startTime
    visitedCount = len(visitedCells)

    print(f"\nRun complete in {durationSeconds:.2f} seconds")
    print(f"Steps taken: {stepCount}")
    print(f"Cells visited: {visitedCount}\n")

    # Save output
    saveMap(mazeMap, stepCount, visitedCount, durationSeconds)

# === ENTRY POINT ===
if __name__ == "__main__":
    robot = Supervisor()
    runRightWallFollower(robot)
