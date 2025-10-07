#!/usr/bin/env python3

# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Curve Fitting Node."""
import csv
import os
import numpy as np
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import yaml
from scipy.interpolate import splprep, splev


class CurveFitting(Node):
    """Curve Fitting Node."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('curve_fitting_node')
        self.declare_parameter('waypoint_file', 'waypoints.csv')
        self.declare_parameter('interpolation_distance', 0.1)
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('plot_results', False)
        self.declare_parameter('output_file', 'cacc_waypoints')
        self.declare_parameter('reverse_order', False)
        self.declare_parameter('is_closed_loop', True)

        self.interpolated_waypoints = []
        self.read_csv()
        self.interpolate_waypoints()
        self.save_interpolated_waypoints()
        self.print_angles_between_waypoints()
        self.plot_interpolated_waypoints()

    def read_csv(self):
        """Read the waypoints from the csv file and swaps if swap_xy is true."""
        self.waypoints = []
        with open(self.get_parameter('waypoint_file').value, 'r', encoding='utf-8-sig') as file:
            reader = csv.reader(file)
            for row in reader:
                if self.get_parameter('swap_xy').value:
                    self.waypoints.append((float(row[1]), float(row[0])))
                else:
                    self.waypoints.append((float(row[0]), float(row[1])))
        
        # Reverse waypoint order if requested
        if self.get_parameter('reverse_order').value:
            self.waypoints.reverse()
            self.get_logger().info('Waypoint order reversed')

    def interpolate_waypoints(self):
        """Interpolate the waypoints using parametric spline for smooth curves."""
        if len(self.waypoints) < 3:
            self.get_logger().error('Need at least 3 waypoints for spline interpolation')
            self.interpolated_waypoints = self.waypoints
            return

        waypoints = np.array(self.waypoints)
        x = waypoints[:, 0]
        y = waypoints[:, 1]

        # Get closed loop setting from parameter
        is_closed_loop = self.get_parameter('is_closed_loop').value
        target_distance = self.get_parameter('interpolation_distance').value
        
        # Parametric spline interpolation
        # s=0 forces the spline to pass through all points (no smoothing)
        # k=3 uses cubic splines for smooth curves
        # per=True creates periodic (closed loop) spline
        try:
            if is_closed_loop:
                # For closed loops, use periodic spline
                tck, u = splprep([x, y], s=0, k=min(3, len(x)-1), per=True)
            else:
                # For open paths, use regular spline
                tck, u = splprep([x, y], s=0, k=min(3, len(x)-1), per=False)
        except Exception as e:
            self.get_logger().error(f'Spline interpolation failed: {e}')
            self.interpolated_waypoints = self.waypoints
            return

        # Calculate total path length for proper point distribution
        num_eval_points = 10000  # High resolution for accurate length calculation
        u_fine = np.linspace(0, 1, num_eval_points)
        x_fine, y_fine = splev(u_fine, tck)
        
        # Calculate cumulative distance along the spline
        dx = np.diff(x_fine)
        dy = np.diff(y_fine)
        distances = np.sqrt(dx**2 + dy**2)
        cumulative_distance = np.concatenate(([0], np.cumsum(distances)))
        total_length = cumulative_distance[-1]
        
        # Calculate number of waypoints based on desired spacing
        num_waypoints = max(int(total_length / target_distance), len(self.waypoints))
        
        # For closed loops, we want to exclude the endpoint (it's the same as start)
        # For open paths, we want to include both endpoints
        if is_closed_loop:
            # Create evenly-spaced points, excluding the endpoint
            target_distances = np.linspace(0, total_length, num_waypoints, endpoint=False)
        else:
            # Include both start and end
            target_distances = np.linspace(0, total_length, num_waypoints, endpoint=True)
        
        # Map distances back to parameter values
        u_interpolated = np.interp(target_distances, cumulative_distance, u_fine)
        
        # Evaluate spline at these parameter values
        x_interpolated, y_interpolated = splev(u_interpolated, tck)
        
        # Store interpolated waypoints
        self.interpolated_waypoints = list(zip(x_interpolated, y_interpolated))
        
        self.get_logger().info(
            f'Interpolated {len(self.waypoints)} waypoints to {len(self.interpolated_waypoints)} points '
            f'(total length: {total_length:.2f}m, spacing: {target_distance:.2f}m, '
            f'closed_loop: {is_closed_loop})'
        )

    def plot_interpolated_waypoints(self):
        """Plot the interpolated waypoints."""
        if not self.get_parameter('plot_results').value:
            return
            
        if not self.interpolated_waypoints:
            self.get_logger().warn('No waypoints to plot')
            return
            
        x_orig, y_orig = zip(*self.waypoints)
        x_interpolated, y_interpolated = zip(*self.interpolated_waypoints)

        plt.figure(figsize=(14, 10))
        
        # Get closed loop setting from parameter
        is_closed = self.get_parameter('is_closed_loop').value
        target_distance = self.get_parameter('interpolation_distance').value
        
        # Plot original waypoints as red markers with larger size
        if is_closed:
            # For closed loops, connect last to first
            plt.plot(list(x_orig) + [x_orig[0]], list(y_orig) + [y_orig[0]], 
                    'r-', linewidth=2, alpha=0.3, label='Original Path', zorder=1)
        plt.plot(x_orig, y_orig, 'ro', markersize=10, label='Original Waypoints', zorder=3)
        
        # Add numbered labels to original waypoints
        for i, (x, y) in enumerate(self.waypoints):
            plt.annotate(str(i), (x, y), 
                        textcoords="offset points", 
                        xytext=(8, 8), 
                        ha='left',
                        fontsize=9, 
                        fontweight='bold',
                        color='darkred',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                                 edgecolor='darkred', alpha=0.8),
                        zorder=6)
        
        # Add arrows for original waypoints to show direction
        for i in range(len(x_orig)):
            next_i = (i + 1) % len(x_orig) if is_closed else min(i + 1, len(x_orig) - 1)
            if i < len(x_orig) - 1 or is_closed:
                dx = x_orig[next_i] - x_orig[i]
                dy = y_orig[next_i] - y_orig[i]
                # Place arrow at midpoint between waypoints
                mid_x = x_orig[i] + dx * 0.5
                mid_y = y_orig[i] + dy * 0.5
                plt.arrow(mid_x, mid_y, dx * 0.15, dy * 0.15, 
                         head_width=0.3, head_length=0.2, fc='darkred', ec='darkred', 
                         alpha=0.7, zorder=4, width=0.05)
        
        # Plot interpolated waypoints as smaller blue dots
        plt.plot(x_interpolated, y_interpolated, 'b.', markersize=3, label='Interpolated Points', zorder=2)
        
        # Draw line connecting interpolated points to show the path
        if is_closed:
            # For closed loops, connect last point back to first
            plt.plot(list(x_interpolated) + [x_interpolated[0]], 
                    list(y_interpolated) + [y_interpolated[0]], 
                    'g-', linewidth=1.5, alpha=0.7, label='Interpolated Path (Closed)', zorder=1)
        else:
            plt.plot(x_interpolated, y_interpolated, 'g-', linewidth=1.5, alpha=0.7, 
                    label='Interpolated Path', zorder=1)
        
        # Add direction arrows for interpolated waypoints (sample every N points to avoid clutter)
        num_waypoints = len(x_interpolated)
        # Show approximately 20-30 arrows regardless of total waypoint count
        arrow_step = max(1, num_waypoints // 25)
        
        for i in range(0, num_waypoints, arrow_step):
            next_i = (i + 1) % num_waypoints if is_closed else min(i + 1, num_waypoints - 1)
            if i < num_waypoints - 1 or is_closed:
                dx = x_interpolated[next_i] - x_interpolated[i]
                dy = y_interpolated[next_i] - y_interpolated[i]
                # Normalize and scale the arrow
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    scale = min(0.5, target_distance * 2)  # Scale based on waypoint spacing
                    plt.arrow(x_interpolated[i], y_interpolated[i], 
                             dx * scale / length, dy * scale / length,
                             head_width=0.15, head_length=0.1, fc='blue', ec='blue', 
                             alpha=0.6, zorder=5, width=0.03)

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        title = f'Spline Interpolation: {len(self.waypoints)} → {len(self.interpolated_waypoints)} waypoints'
        if is_closed:
            title += ' (Closed Loop)'
        if self.get_parameter('reverse_order').value:
            title += ' [REVERSED]'
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')  # Equal aspect ratio to see the true shape
        plt.tight_layout()
        plt.show()

    def save_interpolated_waypoints(self):
        """Save the interpolated waypoints to a YAML file."""
        # Get the input file path and construct the output path
        input_file = self.get_parameter('waypoint_file').value
        input_dir = os.path.dirname(input_file)
        
        # Replace 'install' with 'src' in the path to save to source directory
        if '/install/' in input_dir:
            input_dir = input_dir.replace('/install/', '/src/')
            # Also remove the nested package path structure from install
            # e.g., /install/auna_waypoints/share/auna_waypoints/ -> /src/auna_waypoints/
            parts = input_dir.split('/')
            if 'auna_waypoints' in parts:
                pkg_idx = parts.index('auna_waypoints')
                # Keep everything up to and including the first occurrence of package name
                input_dir = '/'.join(parts[:pkg_idx+1])
                # Add back the config subdirectory
                remaining = '/'.join(parts[pkg_idx+1:])
                if remaining.startswith('share/auna_waypoints/'):
                    remaining = remaining.replace('share/auna_waypoints/', '')
                input_dir = os.path.join(input_dir, remaining)
        
        output_filename = self.get_parameter('output_file').value
        output_file = os.path.join(input_dir, f'{output_filename}.yaml')

        # Convert waypoints to YAML format
        waypoints_yaml = []
        for i, (x, y) in enumerate(self.interpolated_waypoints):
            # Calculate yaw from current point to next point (or to first point if last)
            next_idx = (i + 1) % len(self.interpolated_waypoints)
            next_x, next_y = self.interpolated_waypoints[next_idx]
            yaw = np.arctan2(next_y - y, next_x - x)
            
            # Convert yaw to quaternion
            # For 2D navigation, only z-axis rotation (yaw) is needed
            # Quaternion for rotation around z-axis: q = [0, 0, sin(yaw/2), cos(yaw/2)]
            qz = np.sin(yaw / 2.0)
            qw = np.cos(yaw / 2.0)
            
            waypoint = {
                'position': {
                    'x': float(x),
                    'y': float(y),
                    'z': 0.0
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': float(qz),
                    'w': float(qw)
                }
            }
            waypoints_yaml.append(waypoint)

        with open(output_file, 'w', encoding='utf-8') as file:
            yaml.dump(waypoints_yaml, file, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Saved {len(waypoints_yaml)} interpolated waypoints to: {output_file}')

    def calculate_angle(self, point1, point2):
        """Calculate the angle between two points."""
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return np.arctan2(delta_y, delta_x)

    def print_angles_between_waypoints(self):
        """Calculate the angles between consecutive waypoints."""
        if len(self.interpolated_waypoints) < 3:
            self.get_logger().info('Not enough waypoints to calculate angles.')
            return

        self.get_logger().info('Angles between consecutive waypoints:')

        # Calculate the angle between the first two waypoints as the starting angle
        start_angle = self.calculate_angle(
            self.interpolated_waypoints[0], self.interpolated_waypoints[1]
        )
        self.get_logger().info(f'Waypoint 0 to 1: {np.degrees(start_angle):.2f} degrees')

        # Calculate the angle between the first and second waypoints as the previous angle
        previous_angle = self.calculate_angle(
            self.interpolated_waypoints[1], self.interpolated_waypoints[2]
        )

        # Iterate from the third waypoint onwards
        for i in range(2, len(self.interpolated_waypoints) - 1):
            current_point = self.interpolated_waypoints[i]
            next_point = self.interpolated_waypoints[i + 1]

            # Calculate the angle between the current and next waypoints
            angle = self.calculate_angle(current_point, next_point)

            # Calculate the relative angle with respect to the angle of the two points before
            relative_angle = angle - previous_angle

            # Adjust the relative angle if the jump from 0 to 360 degrees happens
            if relative_angle > np.pi:
                relative_angle -= 2 * np.pi
            elif relative_angle < -np.pi:
                relative_angle += 2 * np.pi

            self.get_logger().info(
                f'Waypoint {i} to {i + 1}: {np.degrees(relative_angle):.2f} degrees')

            # Update the previous angle for the next iteration
            previous_angle = angle


def main(args=None):
    rclpy.init(args=args)
    CurveFitting()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
