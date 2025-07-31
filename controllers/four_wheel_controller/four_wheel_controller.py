"""
Simplified Four-Wheel Robot Controller for Autonomous Maze Navigation

This module provides a clean, simplified interface for maze navigation using
the consolidated robot hardware interface and navigation algorithms. It serves
as the main entry point for autonomous maze solving operations.

Usage Example:
    controller = MazeController()
    controller.initialize()
    controller.wait_for_gps()
    
    # Run different algorithms
    controller.run_dfs()
    controller.run_left_wall_following()
    controller.run_right_wall_following()
    controller.run_smart_wall_following()

Author: Keshav Chaurasia, Mark, Chris, David
Date: July 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

from maze_robot import MazeRobot
from navigation_algorithms import NavigationAlgorithms
from maze_visualizer import MazeVisualizer
import time


class MazeController:
    """
    Maze Controller
    
    This class provides a clean, easy-to-use interface for maze navigation
    by coordinating the robot hardware, navigation algorithms, and visualization
    components. It simplifies the original complex controller architecture.
    """
    
    def __init__(self):
        """Initialize the simplified maze controller."""
        self.robot = MazeRobot()
        self.visualizer = MazeVisualizer()
        self.algorithms = None  # Will be initialized after robot setup
        self.results_history = []
    
    def initialize(self):
        """
        Initialize the maze controller and all subsystems.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        print("=== Initializing Simplified Maze Controller ===")
        
        # Initialize robot hardware
        if not self.robot.initialize():
            print("ERROR: Failed to initialize robot hardware!")
            return False
        
        # Initialize navigation algorithms
        self.algorithms = NavigationAlgorithms(self.robot, self.visualizer)
        
        print("Maze Controller ready!")
        print("Available methods:")
        print("  - run_dfs()")
        print("  - run_left_wall_following()")
        print("  - run_right_wall_following()")
        print("  - run_smart_wall_following()")
        
        return True
    
    def wait_for_gps(self):
        """
        Wait for GPS to provide valid position data.
        
        Returns:
            tuple or None: Starting position if successful, None if failed
        """
        return self.robot.wait_for_gps()
    
    def run_dfs(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Depth-First Search algorithm.
        
        Args:
            start_cell (tuple): Starting cell, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        if not self.algorithms:
            print("ERROR: Controller not initialized!")
            return False
        
        start_time = time.time()
        result = self.algorithms.run_dfs(start_cell, end_cell)
        execution_time = time.time() - start_time
        
        # Store result for comparison
        result_data = {
            'algorithm': 'DFS',
            'success': result,
            'execution_time': execution_time
        }
        self.results_history.append(result_data)
        
        return result
    
    def run_left_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Left Wall Following algorithm.
        
        Args:
            start_cell (tuple): Starting cell, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        if not self.algorithms:
            print("ERROR: Controller not initialized!")
            return False
        
        start_time = time.time()
        result = self.algorithms.run_left_wall_following(start_cell, end_cell)
        execution_time = time.time() - start_time
        
        result_data = {
            'algorithm': 'Left Wall Following',
            'success': result,
            'execution_time': execution_time
        }
        self.results_history.append(result_data)
        
        return result
    
    def run_right_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Right Wall Following algorithm.
        
        Args:
            start_cell (tuple): Starting cell, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        if not self.algorithms:
            print("ERROR: Controller not initialized!")
            return False
        
        start_time = time.time()
        result = self.algorithms.run_right_wall_following(start_cell, end_cell)
        execution_time = time.time() - start_time
        
        result_data = {
            'algorithm': 'Right Wall Following',
            'success': result,
            'execution_time': execution_time
        }
        self.results_history.append(result_data)
        
        return result
    
    def run_smart_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Smart Wall Following algorithm.
        
        Args:
            start_cell (tuple): Starting cell, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        if not self.algorithms:
            print("ERROR: Controller not initialized!")
            return False
        
        start_time = time.time()
        result = self.algorithms.run_smart_wall_following(start_cell, end_cell)
        execution_time = time.time() - start_time
        
        result_data = {
            'algorithm': 'Smart Wall Following',
            'success': result,
            'execution_time': execution_time
        }
        self.results_history.append(result_data)
        
        return result
    
    def get_current_position(self):
        """Get current robot position information."""
        gps_pos = self.robot.get_current_position()
        cell_pos = self.robot.get_current_cell()
        
        return {
            'gps': gps_pos,
            'cell': cell_pos
        }
    
    def get_sensor_readings(self):
        """Get current sensor readings."""
        return self.robot.get_sensor_readings()
    
    def stop(self):
        """Stop the robot and clean up."""
        self.robot.stop()
        print("Robot stopped.")
    
# Main execution for testing
if __name__ == "__main__":
    print("Maze Controller")
    print("This module provides a clean interface for maze navigation.")
    print("Import this module and use MazeController class.")
    
    # Example usage
    controller = MazeController()
    controller.initialize()
    controller.wait_for_gps()
    controller.run_right_wall_following()
    controller.stop()