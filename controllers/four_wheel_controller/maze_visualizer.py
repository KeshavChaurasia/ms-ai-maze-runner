"""
Maze Visualization and Results Module

This module provides visualization, logging, and result analysis capabilities
for maze navigation algorithms. It handles progress tracking, result formatting,
and performance analysis for academic and research purposes.

Key Features:
- Real-time progress visualization during navigation
- Comprehensive result analysis and reporting
- Academic-grade performance metrics
- File-based result logging with timestamps
- ASCII-based maze visualization

Author: Keshav Chaurasia, Mark, Chris, David
Date: July 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

import datetime
import os
import time


class MazeVisualizer:
    """
    Visualization and Results Manager for Maze Navigation
    
    This class provides comprehensive visualization and analysis capabilities
    for maze navigation algorithms, including real-time progress tracking,
    result formatting, and performance metrics calculation.
    """
    
    def __init__(self, results_dir="maze_results"):
        """
        Initialize the maze visualizer.
        
        Args:
            results_dir (str): Directory to save result files
        """
        self.results_dir = results_dir
        self._ensure_results_directory()
    
    def _ensure_results_directory(self):
        """Create results directory if it doesn't exist."""
        if not os.path.exists(self.results_dir):
            try:
                os.makedirs(self.results_dir)
                print(f"Created results directory: {self.results_dir}")
            except:
                print(f"Warning: Could not create results directory: {self.results_dir}")
    
    def print_progress(self, visited_cells, current_cell, step_count):
        """
        Print current navigation progress.
        
        Args:
            visited_cells (set): Set of visited cell coordinates
            current_cell (tuple): Current robot position
            step_count (int): Current step number
        """
        print(f"\n=== PROGRESS UPDATE - Step {step_count} ===")
        print(f"Current position: {current_cell}")
        print(f"Cells explored: {len(visited_cells)}")
        print(f"Coverage: {len(visited_cells)/144*100:.1f}% of maze")
        self.print_visited_cells(visited_cells)
    
    def print_visited_cells(self, visited_cells):
        """
        Print ASCII visualization of visited cells.
        
        Args:
            visited_cells (set): Set of visited cell coordinates
        """
        print("\nMaze Progress (X = visited, . = unvisited):")
        print("  " + "".join([f"{i:2d}" for i in range(12)]))  # Column headers
        
        for row in range(11, -1, -1):  # Top to bottom
            line = f"{row:2d} "
            for col in range(12):  # Left to right
                if (col, row) in visited_cells:
                    line += "X "
                else:
                    line += ". "
            print(line)
    
    def print_results(self, result, visited_cells, path_stack=None):
        """
        Print comprehensive algorithm results.
        
        Args:
            result (dict): Algorithm result data
            visited_cells (set): Set of visited cells
            path_stack (list, optional): Final path for DFS
        """
        algorithm = result['algorithm']
        success = result['success']
        
        print(f"\n{'='*60}")
        print(f"FINAL RESULTS - {algorithm}")
        print(f"{'='*60}")
        
        if success:
            print(f"🎉 SUCCESS! Reached target cell {result.get('final_position', 'Unknown')}")
        else:
            print(f"❌ FAILED to reach target after {result['steps']} steps")
            print(f"Final position: {result.get('final_position', 'Unknown')}")
        
        print(f"\nPerformance Metrics:")
        print(f"  Algorithm: {algorithm}")
        print(f"  Success: {'YES' if success else 'NO'}")
        print(f"  Total steps: {result['steps']}")
        print(f"  Cells visited: {result['visited_count']}")
        print(f"  Maze coverage: {result['visited_count']/144*100:.1f}%")
        
        if path_stack:
            print(f"  Final path length: {len(path_stack)} cells")
            print(f"  Path efficiency: {len(path_stack)/result['steps']*100:.1f}%")
        
        if 'path_efficiency' in result:
            print(f"  Path efficiency: {result['path_efficiency']:.1f}%")
        
        print(f"\nExecution Details:")
        print(f"  Start time: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  Steps per second: {result['steps']/max(1, result.get('duration', 1)):.1f}")
        
        # Print final maze state
        print(f"\nFinal Maze State:")
        self.print_visited_cells(visited_cells)
        
        if path_stack:
            print(f"\nSolution Path:")
            print(f"  {' -> '.join(map(str, path_stack))}")
        
        # Save results to file
        self._save_results_to_file(result, visited_cells, path_stack)
    
    def _save_results_to_file(self, result, visited_cells, path_stack=None):
        """
        Save detailed results to a timestamped file.
        
        Args:
            result (dict): Algorithm result data
            visited_cells (set): Set of visited cells
            path_stack (list, optional): Final path for DFS
        """
        try:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            algorithm_name = result['algorithm'].replace(' ', '_')
            filename = f"maze_result_{algorithm_name}_{timestamp}.txt"
            filepath = os.path.join(self.results_dir, filename)
            
            with open(filepath, 'w') as f:
                f.write(f"Maze Navigation Results\n")
                f.write(f"{'='*50}\n\n")
                
                f.write(f"Algorithm: {result['algorithm']}\n")
                f.write(f"Timestamp: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Success: {'YES' if result['success'] else 'NO'}\n")
                f.write(f"Final Position: {result.get('final_position', 'Unknown')}\n\n")
                
                f.write(f"Performance Metrics:\n")
                f.write(f"  Total Steps: {result['steps']}\n")
                f.write(f"  Cells Visited: {result['visited_count']}\n")
                f.write(f"  Maze Coverage: {result['visited_count']/144*100:.1f}%\n")
                
                if path_stack:
                    f.write(f"  Path Length: {len(path_stack)}\n")
                    f.write(f"  Path Efficiency: {len(path_stack)/result['steps']*100:.1f}%\n")
                
                if 'path_efficiency' in result:
                    f.write(f"  Path Efficiency: {result['path_efficiency']:.1f}%\n")
                
                f.write(f"\nVisited Cells ({len(visited_cells)} total):\n")
                visited_list = sorted(list(visited_cells))
                for i, cell in enumerate(visited_list):
                    if i % 10 == 0:
                        f.write("\n  ")
                    f.write(f"{cell} ")
                f.write("\n")
                
                if path_stack:
                    f.write(f"\nSolution Path ({len(path_stack)} cells):\n")
                    f.write(f"  {' -> '.join(map(str, path_stack))}\n")
                
                f.write(f"\nMaze Visualization:\n")
                f.write("  " + "".join([f"{i:2d}" for i in range(12)]) + "\n")
                for row in range(11, -1, -1):
                    line = f"{row:2d} "
                    for col in range(12):
                        if (col, row) in visited_cells:
                            line += "X "
                        else:
                            line += ". "
                    f.write(line + "\n")
            
            print(f"\nResults saved to: {filepath}")
            
        except Exception as e:
            print(f"Warning: Could not save results to file: {e}")
    
    def generate_comparison_report(self, results_list):
        """
        Generate comparison report for multiple algorithm runs.
        
        Args:
            results_list (list): List of result dictionaries from different algorithms
        """
        print(f"\n{'='*80}")
        print("ALGORITHM COMPARISON REPORT")
        print(f"{'='*80}")
        
        if not results_list:
            print("No results to compare.")
            return
        
        # Header
        print(f"{'Algorithm':<25} {'Success':<8} {'Steps':<8} {'Coverage':<10} {'Efficiency':<12}")
        print(f"{'-'*25} {'-'*8} {'-'*8} {'-'*10} {'-'*12}")
        
        # Results
        for result in results_list:
            algorithm = result['algorithm'][:24]  # Truncate if too long
            success = "YES" if result['success'] else "NO"
            steps = result['steps']
            coverage = f"{result['visited_count']/144*100:.1f}%"
            
            efficiency = "N/A"
            if 'path_efficiency' in result:
                efficiency = f"{result['path_efficiency']:.1f}%"
            elif result.get('path_length'):
                efficiency = f"{result['path_length']/result['steps']*100:.1f}%"
            
            print(f"{algorithm:<25} {success:<8} {steps:<8} {coverage:<10} {efficiency:<12}")
        
        # Summary statistics
        successful_runs = [r for r in results_list if r['success']]
        if successful_runs:
            avg_steps = sum(r['steps'] for r in successful_runs) / len(successful_runs)
            avg_coverage = sum(r['visited_count'] for r in successful_runs) / len(successful_runs)
            
            print(f"\nSummary (successful runs only):")
            print(f"  Success rate: {len(successful_runs)}/{len(results_list)} ({len(successful_runs)/len(results_list)*100:.1f}%)")
            print(f"  Average steps: {avg_steps:.1f}")
            print(f"  Average coverage: {avg_coverage:.1f} cells ({avg_coverage/144*100:.1f}%)")
    
    def create_performance_visualization(self, visited_cells, title="Maze Navigation"):
        """
        Create an enhanced ASCII visualization with performance info.
        
        Args:
            visited_cells (set): Set of visited cell coordinates
            title (str): Title for the visualization
        """
        print(f"\n{title}")
        print("=" * len(title))
        
        # Statistics
        total_cells = 144
        visited_count = len(visited_cells)
        coverage_percent = visited_count / total_cells * 100
        
        print(f"Coverage: {visited_count}/{total_cells} cells ({coverage_percent:.1f}%)")
        
        # Enhanced visualization with borders
        print("\n┌" + "─" * 25 + "┐")
        print("│  " + "".join([f"{i:2d}" for i in range(12)]) + " │")
        print("├" + "─" * 25 + "┤")
        
        for row in range(11, -1, -1):
            line = f"│{row:2d} "
            for col in range(12):
                if (col, row) in visited_cells:
                    line += "█ "  # Solid block for visited
                else:
                    line += "░ "  # Light shade for unvisited
            line += "│"
            print(line)
        
        print("└" + "─" * 25 + "┘")
        
        # Legend
        print("\nLegend: █ = Visited, ░ = Unvisited")
    
    def time_algorithm_execution(self, algorithm_func, *args, **kwargs):
        """
        Time the execution of an algorithm and return results with timing info.
        
        Args:
            algorithm_func: Function to execute
            *args: Arguments for the function
            **kwargs: Keyword arguments for the function
            
        Returns:
            tuple: (result, execution_time)
        """
        start_time = time.time()
        result = algorithm_func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        
        print(f"\nExecution Time: {execution_time:.2f} seconds")
        
        return result, execution_time
