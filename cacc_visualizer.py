#!/usr/bin/env python3

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec


class CACCVisualizer:
    def __init__(self, log_file):
        self.log_file = log_file
        self.data = None
        self.load_data()

    def load_data(self):
        """Load data from CSV file"""
        print(f"Loading data from {self.log_file}...")
        try:
            self.data = pd.read_csv(self.log_file)
            # Convert timestamp to relative time
            if 'timestamp' in self.data.columns:
                self.data['rel_time'] = self.data['timestamp'] - \
                    self.data['timestamp'].iloc[0]
            print(f"Loaded {len(self.data)} data points")
        except Exception as e:
            print(f"Error loading data: {e}")
            exit(1)

    def create_static_plots(self):
        """Create static plots of the CACC data"""
        if self.data is None or len(self.data) == 0:
            print("No data to visualize")
            return

        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('CACC Controller Performance', fontsize=16)
        gs = gridspec.GridSpec(3, 2)

        # 1. Velocities plot
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(self.data['rel_time'],
                 self.data['leader_velocity'], 'b-', label='Leader')
        ax1.plot(self.data['rel_time'],
                 self.data['follower_velocity'], 'r-', label='Follower')
        ax1.plot(self.data['rel_time'],
                 self.data['commanded_velocity'], 'g--', label='Commanded')
        ax1.set_title('Velocities')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.legend()
        ax1.grid(True)

        # 2. Accelerations plot
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(self.data['rel_time'],
                 self.data['leader_acceleration'], 'b-', label='Leader')
        ax2.plot(self.data['rel_time'],
                 self.data['follower_acceleration'], 'r-', label='Follower')
        ax2.plot(self.data['rel_time'],
                 self.data['commanded_accel'], 'g--', label='Commanded')
        ax2.set_title('Accelerations')
        ax2.set_ylabel('Acceleration (m/s²)')
        ax2.legend()
        ax2.grid(True)

        # 3. Distance and errors plot
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(self.data['rel_time'],
                 self.data['desired_distance'], 'b--', label='Desired')
        ax3.plot(self.data['rel_time'],
                 self.data['actual_distance'], 'r-', label='Actual')
        ax3.set_title('Following Distance')
        ax3.set_ylabel('Distance (m)')
        ax3.legend()
        ax3.grid(True)

        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(self.data['rel_time'], self.data['distance_error'], 'r-')
        ax4.set_title('Distance Error')
        ax4.set_ylabel('Error (m)')
        ax4.grid(True)

        # 4. Error state plots
        ax5 = fig.add_subplot(gs[2, 0])
        ax5.plot(self.data['rel_time'], self.data['z1'],
                 'r-', label='z1 (longitudinal pos)')
        ax5.plot(self.data['rel_time'], self.data['z2'],
                 'b-', label='z2 (lateral pos)')
        ax5.set_title('Position Error States')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Error (m)')
        ax5.legend()
        ax5.grid(True)

        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(self.data['rel_time'], self.data['z3'],
                 'r-', label='z3 (longitudinal vel)')
        ax6.plot(self.data['rel_time'], self.data['z4'],
                 'b-', label='z4 (lateral vel)')
        ax6.set_title('Velocity Error States')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Error (m/s)')
        ax6.legend()
        ax6.grid(True)

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.show()

    def create_vehicle_animation(self):
        """Create animation of vehicle positions"""
        if self.data is None or len(self.data) == 0:
            print("No data to visualize")
            return

        # Setup the figure
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title('Vehicle Positions')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')

        # Get max and min values for x and y to set proper limits
        min_x = min(self.data['leader_x'].min(), self.data['follower_x'].min())
        max_x = max(self.data['leader_x'].max(), self.data['follower_x'].max())
        min_y = min(self.data['leader_y'].min(), self.data['follower_y'].min())
        max_y = max(self.data['leader_y'].max(), self.data['follower_y'].max())

        # Add some margin
        margin = max(max_x - min_x, max_y - min_y) * 0.1
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)

        # Create path lines
        leader_path, = ax.plot([], [], 'b-', alpha=0.3, label='Leader path')
        follower_path, = ax.plot(
            [], [], 'r-', alpha=0.3, label='Follower path')

        # Create markers for vehicles
        leader_pos, = ax.plot([], [], 'bo', markersize=8, label='Leader')
        follower_pos, = ax.plot([], [], 'ro', markersize=8, label='Follower')

        # Create a status text
        status_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

        ax.grid(True)
        ax.legend(loc='upper right')

        # Downsample data for animation to make it smoother
        step = max(1, len(self.data) // 300)
        downsampled_data = self.data.iloc[::step]

        def init():
            leader_path.set_data([], [])
            follower_path.set_data([], [])
            leader_pos.set_data([], [])
            follower_pos.set_data([], [])
            status_text.set_text('')
            return leader_path, follower_path, leader_pos, follower_pos, status_text

        def update(frame):
            idx = frame

            # Update paths
            leader_path.set_data(
                downsampled_data['leader_x'][:idx], downsampled_data['leader_y'][:idx])
            follower_path.set_data(
                downsampled_data['follower_x'][:idx], downsampled_data['follower_y'][:idx])

            # Update current positions
            leader_pos.set_data(
                downsampled_data['leader_x'][idx], downsampled_data['leader_y'][idx])
            follower_pos.set_data(
                downsampled_data['follower_x'][idx], downsampled_data['follower_y'][idx])

            # Update status text
            time_str = f"Time: {downsampled_data['rel_time'].iloc[idx]:.2f}s"
            vel_str = f"Velocities - L: {downsampled_data['leader_velocity'].iloc[idx]:.2f}, F: {downsampled_data['follower_velocity'].iloc[idx]:.2f} m/s"
            dist_str = f"Distance: {downsampled_data['actual_distance'].iloc[idx]:.2f}m (error: {downsampled_data['distance_error'].iloc[idx]:.2f}m)"
            status_text.set_text(f"{time_str}\n{vel_str}\n{dist_str}")

            return leader_path, follower_path, leader_pos, follower_pos, status_text

        ani = FuncAnimation(
            fig, update, frames=len(downsampled_data),
            init_func=init, blit=True, interval=50, repeat=True
        )

        plt.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='CACC Data Visualizer')
    parser.add_argument('log_file', help='Path to the CACC log CSV file')
    parser.add_argument('--static', action='store_true',
                        help='Generate static plots only')
    parser.add_argument('--animation', action='store_true',
                        help='Generate animation plot only')

    args = parser.parse_args()

    visualizer = CACCVisualizer(args.log_file)

    # Default: show both if no specific option is selected
    show_static = args.static or not args.animation
    show_animation = args.animation or not args.static

    if show_static:
        visualizer.create_static_plots()

    if show_animation:
        visualizer.create_vehicle_animation()


if __name__ == "__main__":
    main()
