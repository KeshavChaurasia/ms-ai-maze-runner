# Behaviour-based Map Building Robotic System: Micromouse Maze Solver

## Project Overview
This project implements a behaviour-based robotic system to solve a maze using Webots simulation. The robot autonomously navigates a rectangular maze, detects obstacles, builds a map of its environment, and finds a path from the start to the finish position. The project is inspired by the Micromouse competition, where small robots solve mazes as quickly as possible.

## Features
- Rectangular maze environment (at least 4x4 cells) with wall boundaries and internal obstacles
- Autonomous robot navigation from start to finish
- Obstacle detection using distance/vision sensors
- Map building: generates a matrix representation of the explored maze
- Visualization of the robot's path, obstacles, and unexplored regions
- Competition time measurement
- Experimentation with different sensors and algorithms

## Project Structure
- `worlds/maze.wbt`: Webots world file defining the maze environment
- `controllers/`: (To be added) Robot controller code
- `protos/`: (Optional) Custom robot or sensor definitions
- `plugins/`: (Optional) Custom plugins for physics, remote control, or robot windows

## Setup Instructions
1. **Install Webots:**
   - Download and install Webots from [https://cyberbotics.com/](https://cyberbotics.com/)
2. **Clone this repository:**
   ```bash
   git clone <repo-url>
   cd ms-ai-maze-runner
   ```
3. **Open the project in Webots:**
   - Launch Webots
   - Open the `worlds/maze.wbt` world file
4. **Build/Configure the Controller:**
   - (To be added) Add or compile the robot controller in the `controllers/` directory

## Usage
1. Start the simulation in Webots.
2. The robot will begin navigating the maze from the start position.
3. The robot uses its sensors to detect obstacles and build a map as it explores.
4. The simulation ends when the robot reaches the finish position or after a set time limit.
5. The resulting map will show:
   - `1`: Obstacle
   - `0`: Free space
   - `?`: Unexplored region
   - (Optional) The robot's travel path highlighted

## Robot and Sensors
- **Robot:** (To be specified, e.g., e-puck or custom robot)
- **Sensors:** (To be specified, e.g., 4 distance sensors on each side, or vision sensors)
- **Actuators:** (To be specified, e.g., differential wheels)

## Maze-Solving Algorithm
- (To be specified, e.g., right-hand rule, A*, priority-based movement)
- The robot prioritizes moving straight, then right, then left, then back if blocked
- The robot updates its internal map matrix as it moves

## Map Building
- The robot maintains a matrix representation of the maze
- Example:
  ```
  1 1 1 1 1
  1 0 0 1 1
  1 0 ? 0 1
  1 1 1 1 1
  ```
- After the simulation, the path taken by the robot is marked (e.g., in red or with a special symbol)

## Competition and Performance
- The time taken to solve the maze is measured
- Discussion of techniques or algorithms to improve performance (e.g., sensor placement, algorithm optimization)
- (Optional) Results from experiments with different sensors or algorithms

## Video Presentation
- (To be added) A 10-minute video summarizing the project, map building, and results

## Authors
- (To be filled by group members)

## References
- Webots documentation: https://cyberbotics.com/doc/
- Micromouse competition: https://en.wikipedia.org/wiki/Micromouse

---
*This README is a draft template. Please update the robot, sensors, algorithms, and results sections as your project progresses.*
