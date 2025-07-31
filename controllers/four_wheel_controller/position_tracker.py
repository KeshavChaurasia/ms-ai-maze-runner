"""
Position Tracking Module for Robot Navigation System

This module provides comprehensive position tracking and coordinate system
conversion capabilities for autonomous robot navigation. It interfaces with
GPS hardware to obtain real-time position data and handles the mathematical
transformations between different coordinate reference frames.

Key Functionality:
- Real-time GPS position acquisition and validation
- Coordinate system transformations (GPS ↔ Grid Cell)
- Data integrity checking and error handling
- Support for discrete grid-based navigation systems

Coordinate Systems:
1. GPS Coordinate System: Continuous real-world coordinates in meters
   - Origin varies based on simulation/hardware setup
   - Provides sub-meter precision for accurate navigation
   
2. Cell Coordinate System: Discrete grid-based coordinates
   - Integer values representing maze cell positions
   - Simplified representation for pathfinding algorithms
   - Origin typically at (0,0) for the first navigable cell

Mathematical Foundation:
The coordinate transformations assume a standard grid layout where each
cell occupies a 1x1 meter square, with cell centers offset by 0.5 meters
from the grid boundaries. This design facilitates precise robot positioning
within maze cells while maintaining computational simplicity.

Author: [Your Name]
Date: July 2025
Academic Project: AI-Powered Maze Navigation with Four-Wheel Robot
"""

import math


class PositionTracker:
    """
    GPS-Based Position Tracking and Coordinate Conversion System
    
    This class provides a comprehensive interface for robot position tracking
    using GPS hardware. It manages real-time position acquisition, data
    validation, and coordinate system transformations required for autonomous
    navigation in grid-based environments.
    
    Design Philosophy:
    The tracker abstracts GPS hardware complexity while providing reliable
    position data and coordinate transformations. It implements robust error
    handling to ensure system stability when GPS signals are unreliable.
    
    Coordinate Reference Frames:
    - GPS Frame: Continuous coordinates in meters (real-world positions)
    - Cell Frame: Discrete grid coordinates (integer cell indices)
    
    The class handles bidirectional transformations between these frames,
    enabling seamless integration between high-level path planning
    (cell-based) and low-level motion control (GPS-based).
    """
    
    
    def __init__(self, gps_device):
        """
        Initialize the Position Tracker with GPS hardware interface.
        
        Establishes connection to the GPS device and prepares the tracking
        system for position acquisition. The GPS device should be properly
        configured and enabled before passing to this constructor.
        
        Args:
            gps_device: Webots GPS device object obtained from robot.getDevice()
                       Must be a properly initialized GPS sensor with adequate
                       sampling rate for real-time navigation requirements
                       
        Note:
            The GPS device should be enabled with appropriate sampling
            frequency before use. Typical navigation applications require
            sampling rates of 10-100 Hz for smooth motion control.
        """
        # Store reference to GPS hardware interface
        # This device provides continuous position updates during navigation
        self.gps = gps_device
    
    
    def get_current_position(self):
        """
        Retrieve current robot position with data validation.
        
        Obtains real-time GPS coordinates and performs data integrity checking
        to ensure reliable position information. This method handles common
        GPS signal issues such as temporary signal loss or initialization delays.
        
        Data Validation Process:
        1. Acquire raw GPS readings from hardware
        2. Extract X and Y coordinates (ignore Z for 2D navigation)
        3. Check for NaN values indicating signal problems
        4. Return validated coordinates or None for invalid data
        
        Returns:
            tuple or None: (x, y) coordinates in meters if valid GPS signal,
                          None if GPS signal is invalid or unavailable
                          
        Error Conditions:
        - GPS device not properly initialized
        - Temporary signal loss or interference
        - Hardware malfunction or disconnection
        - System startup before GPS acquisition
        
        Usage Example:
            >>> pos = tracker.get_current_position()
            >>> if pos is not None:
            >>>     x, y = pos
            >>>     print(f"Robot at: ({x:.3f}, {y:.3f})")
            >>> else:
            >>>     print("GPS signal unavailable")
        """
        # Acquire raw GPS data from hardware device
        # getValues() returns [x, y, z] array; we only need x, y for 2D navigation
        position = self.gps.getValues()[:2]
        
        # Data integrity check: verify both coordinates are valid numbers
        # NaN values indicate GPS signal problems or initialization issues
        if math.isnan(position[0]) or math.isnan(position[1]):
            # Invalid GPS data - return None to signal error condition
            # Calling code should handle this gracefully (retry, use odometry, etc.)
            return None
        
        # Return validated position data as tuple
        return position
    
    
    def gps_to_cell(self, gps_x, gps_y):
        """
        Convert continuous GPS coordinates to discrete cell coordinates.
        
        Transforms real-world GPS positions into integer-based grid cell
        indices suitable for discrete path planning algorithms. This conversion
        is essential for interfacing between continuous motion control and
        discrete maze representation.
        
        Mathematical Transformation:
        Cell coordinates are calculated using the formula:
        cell_coord = round(gps_coord - 0.5)
        
        This formula assumes:
        - Each cell occupies a 1×1 meter square
        - Cell centers are located at (n+0.5, m+0.5) in GPS coordinates
        - Grid origin (0,0) corresponds to GPS coordinates (0.5, 0.5)
        
        Coordinate System Mapping:
        GPS Range [0.0, 1.0) → Cell 0
        GPS Range [1.0, 2.0) → Cell 1
        GPS Range [2.0, 3.0) → Cell 2
        And so forth...
        
        Args:
            gps_x (float): GPS X-coordinate in meters
            gps_y (float): GPS Y-coordinate in meters
            
        Returns:
            tuple: (cell_x, cell_y) as integer cell indices
            
        Example:
            >>> # Robot at GPS position (2.7, 1.3)
            >>> cell_coords = tracker.gps_to_cell(2.7, 1.3)
            >>> print(cell_coords)  # Output: (2, 1)
            
        Note:
            This method performs rounding rather than truncation to ensure
            that positions near cell boundaries are assigned to the nearest
            cell center, improving navigation accuracy.
        """
        # Apply coordinate transformation with 0.5 offset compensation
        # Round to nearest integer to handle positions near cell boundaries
        return round(gps_x - 0.5), round(gps_y - 0.5)
    
    
    def cell_to_gps(self, cell_x, cell_y):
        """
        Convert discrete cell coordinates to continuous GPS coordinates.
        
        Transforms integer-based grid cell indices into real-world GPS
        positions suitable for precise motion control. This conversion
        targets the geometric center of each cell for optimal robot
        positioning within the maze structure.
        
        Mathematical Transformation:
        GPS coordinates are calculated using the formula:
        gps_coord = cell_coord + 0.5
        
        This formula ensures:
        - Robot targets the center of each grid cell
        - Consistent 0.5-meter offset from cell boundaries
        - Optimal positioning for maze navigation and obstacle avoidance
        
        Coordinate System Mapping:
        Cell 0 → GPS Position (0.5, 0.5)
        Cell 1 → GPS Position (1.5, 1.5)  
        Cell 2 → GPS Position (2.5, 2.5)
        And so forth...
        
        Args:
            cell_x (int): Cell X-coordinate (grid column index)
            cell_y (int): Cell Y-coordinate (grid row index)
            
        Returns:
            tuple: (gps_x, gps_y) as floating-point GPS coordinates in meters
            
        Example:
            >>> # Target cell (3, 2) in the maze grid
            >>> gps_coords = tracker.cell_to_gps(3, 2)
            >>> print(gps_coords)  # Output: (3.5, 2.5)
            
        Usage in Navigation:
        This method is typically used by path planning algorithms to convert
        high-level waypoints (cell coordinates) into precise target positions
        for the motion control system.
        
        Design Rationale:
        Targeting cell centers rather than corners or edges provides:
        - Maximum clearance from maze walls
        - Reduced risk of collision during navigation
        - Simplified obstacle detection and avoidance
        - Consistent robot orientation within corridors
        """
        # Apply coordinate transformation with 0.5 offset for cell center targeting
        # Addition converts discrete indices to continuous center positions
        return cell_x + 0.5, cell_y + 0.5
