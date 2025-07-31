"""
Configuration Constants for Four-Wheel Robot Maze Navigation System

This module contains all configuration parameters used throughout the robotics
maze runner project. These constants control robot behavior, sensor sensitivity,
movement precision, and debugging output.

Author: Keshav Chaurasia
Date: July 2025
Academic Project: AI-Powered Maze Navigation with Four-Wheel Robot
"""

class Config:
    """
    Central configuration class containing all system parameters.
    
    This class serves as a single source of truth for all configuration
    values used across the maze navigation system, promoting maintainability
    and easy parameter tuning for different maze environments.
    """
    
    # === SIMULATION TIMING PARAMETERS ===
    TIME_STEP = 64 # Simulation time step in milliseconds
                    # Controls the frequency of robot controller updates
                    # Lower values = higher precision but more computational load
                    # Typical range: 32-128ms for real-time performance
    
    # === ROBOT MOVEMENT PARAMETERS ===
    MAX_SPEED = 2.0  # Maximum wheel velocity in radians per second
                     # Determines the robot's maximum linear and angular speeds
                     # Higher values allow faster maze traversal but may reduce
                     # control precision and increase overshoot errors
    
    # === MAZE ENVIRONMENT PARAMETERS ===
    CELL_SIZE = 1.0  # Size of each maze cell in meters
                     # This should match the actual maze cell dimensions
                     # Used for path planning and position calculations
                     # Standard maze cells are typically 1.0m x 1.0m
    
    # === MOVEMENT PRECISION PARAMETERS ===
    MOVEMENT_TOLERANCE = 0.05  # Position tolerance in meters
                               # Defines how close the robot must be to target
                               # before considering a movement complete
                               # Smaller values = higher precision but slower movement
                               # Must balance speed vs. accuracy requirements
    
    # === DEBUGGING AND MONITORING PARAMETERS ===
    DEBUG_FREQUENCY = 25  # Print debug information every N simulation steps
                          # Higher values reduce console output verbosity
                          # Useful for monitoring robot behavior during development
                          # Set to 0 to disable debug output entirely
    
    # === SENSOR CONFIGURATION PARAMETERS ===
    SENSOR_THRESHOLD = 400  # Distance sensor threshold value
                            # Raw sensor reading below this value indicates
                            # an obstacle or wall is detected
                            # Value depends on sensor type and maze wall material
                            # Typically calibrated through experimental testing

