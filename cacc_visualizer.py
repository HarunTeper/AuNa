#!/usr/bin/env python3

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
import os


class CACCVisualizer:
    def __init__(self, cacc_log_file, cam_log_file=None):
        self.cacc_log_file = cacc_log_file
        self.cam_log_file = cam_log_file
        self.cacc_data = None
        self.cam_data = None
        self.load_data()

    def load_data(self):
        """Load data from CSV file(s)"""
        print(f"Loading CACC data from {self.cacc_log_file}...")
        try:
            self.cacc_data = pd.read_csv(self.cacc_log_file)
            # Convert timestamp to relative time
            if 'timestamp' in self.cacc_data.columns:
                self.cacc_data['rel_time'] = self.cacc_data['timestamp'] - \
                    self.cacc_data['timestamp'].iloc[0]
            print(f"Loaded {len(self.cacc_data)} CACC data points")

            # Load CAM data if file path is provided
            if self.cam_log_file and os.path.exists(self.cam_log_file):
                print(f"Loading CAM data from {self.cam_log_file}...")
                try:
                    # Skip the first line (header description) and use the second line as column names
                    with open(self.cam_log_file, 'r') as f:
                        first_line = f.readline()  # Skip the first descriptive line
                        if '===' in first_line:  # This is our header comment
                            self.cam_data = pd.read_csv(
                                self.cam_log_file, skiprows=1)
                        else:  # No header comment, read normally
                            self.cam_data = pd.read_csv(self.cam_log_file)

                    # Convert timestamp to relative time aligned with CACC data
                    if 'Timestamp' in self.cam_data.columns:
                        # Align with CACC data start time if possible
                        start_time = self.cacc_data['timestamp'].iloc[
                            0] if 'timestamp' in self.cacc_data.columns else self.cam_data['Timestamp'].iloc[0]
                        self.cam_data['rel_time'] = self.cam_data['Timestamp'] - start_time

                    print(f"Loaded {len(self.cam_data)} CAM data points")
                except Exception as e:
                    print(f"Error loading CAM data: {e}")
                    self.cam_data = None
            elif self.cam_log_file:
                print(f"CAM log file not found: {self.cam_log_file}")

        except Exception as e:
            print(f"Error loading CACC data: {e}")
            exit(1)

    def create_static_plots(self):
        """Create static plots of the CACC and CAM data"""
        if self.cacc_data is None or len(self.cacc_data) == 0:
            print("No CACC data to visualize")
            return

        # Determine if we're showing only CACC or integrated data
        has_cam_data = self.cam_data is not None and len(self.cam_data) > 0

        if has_cam_data:
            # Create figure with more subplots for integrated data
            fig = plt.figure(figsize=(18, 14))
            fig.suptitle(
                'Integrated CACC & CAM Data Visualization', fontsize=16)
            gs = gridspec.GridSpec(4, 2)
        else:
            # Original CACC-only layout
            fig = plt.figure(figsize=(16, 12))
            fig.suptitle('CACC Controller Performance', fontsize=16)
            gs = gridspec.GridSpec(3, 2)

        # 1. Velocities plot (common for both layouts)
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(self.cacc_data['rel_time'],
                 self.cacc_data['leader_velocity'], 'b-', label='Leader')
        ax1.plot(self.cacc_data['rel_time'],
                 self.cacc_data['follower_velocity'], 'r-', label='Follower')
        ax1.plot(self.cacc_data['rel_time'],
                 self.cacc_data['commanded_velocity'], 'g--', label='Commanded')
        ax1.set_title('Velocities')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.legend()
        ax1.grid(True)

        # 2. Accelerations plot (common for both layouts)
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(self.cacc_data['rel_time'],
                 self.cacc_data['leader_acceleration'], 'b-', label='Leader')
        ax2.plot(self.cacc_data['rel_time'],
                 self.cacc_data['follower_acceleration'], 'r-', label='Follower')
        ax2.plot(self.cacc_data['rel_time'],
                 self.cacc_data['commanded_accel'], 'g--', label='Commanded')
        ax2.set_title('Accelerations')
        ax2.set_ylabel('Acceleration (m/s²)')
        ax2.legend()
        ax2.grid(True)

        # 3. Distance and errors plot (common for both layouts)
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(self.cacc_data['rel_time'],
                 self.cacc_data['desired_distance'], 'b--', label='Desired')
        ax3.plot(self.cacc_data['rel_time'],
                 self.cacc_data['actual_distance'], 'r-', label='Actual')
        ax3.set_title('Following Distance')
        ax3.set_ylabel('Distance (m)')
        ax3.legend()
        ax3.grid(True)

        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(self.cacc_data['rel_time'],
                 self.cacc_data['distance_error'], 'r-')
        ax4.set_title('Distance Error')
        ax4.set_ylabel('Error (m)')
        ax4.grid(True)

        # 4. Error state plots (common for both layouts)
        ax5 = fig.add_subplot(gs[2, 0])
        ax5.plot(self.cacc_data['rel_time'], self.cacc_data['z1'],
                 'r-', label='z1 (longitudinal pos)')
        ax5.plot(self.cacc_data['rel_time'], self.cacc_data['z2'],
                 'b-', label='z2 (lateral pos)')
        ax5.set_title('Position Error States')
        ax5.set_ylabel('Error (m)')
        ax5.legend()
        ax5.grid(True)

        # Only add xlabel if this is the bottom row
        if not has_cam_data:
            ax5.set_xlabel('Time (s)')

        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(self.cacc_data['rel_time'], self.cacc_data['z3'],
                 'r-', label='z3 (longitudinal vel)')
        ax6.plot(self.cacc_data['rel_time'], self.cacc_data['z4'],
                 'b-', label='z4 (lateral vel)')
        ax6.set_title('Velocity Error States')
        ax6.set_ylabel('Error (m/s)')
        ax6.legend()
        ax6.grid(True)

        # Only add xlabel if this is the bottom row
        if not has_cam_data:
            ax6.set_xlabel('Time (s)')

        # Add CAM-specific plots if we have CAM data
        if has_cam_data:
            try:
                # Filter TX and RX actions to plot separately
                cam_tx = self.cam_data[self.cam_data['Action'] == 'TX']
                cam_rx = self.cam_data[self.cam_data['Action'] == 'RX']

                # 5. CAM Message Frequency
                ax7 = fig.add_subplot(gs[3, 0])

                # Group by 1-second windows and count messages
                if not cam_tx.empty:
                    # Use histogram to show distribution of messages over time
                    ax7.hist(cam_tx['rel_time'], bins=min(50, int(max(cam_tx['rel_time']))+1),
                             alpha=0.5, label='TX', color='green')

                if not cam_rx.empty:
                    ax7.hist(cam_rx['rel_time'], bins=min(50, int(max(cam_rx['rel_time']))+1),
                             alpha=0.5, label='RX', color='blue')

                ax7.set_title('CAM Message Distribution')
                ax7.set_xlabel('Time (s)')
                ax7.set_ylabel('Message Count')
                ax7.legend()
                ax7.grid(True)

                # 6. CAM Message Content - Plot some key fields
                ax8 = fig.add_subplot(gs[3, 1])

                # Plot curvature from CAM messages over time
                if 'Curvature' in self.cam_data.columns and not cam_tx.empty:
                    # For numeric curvature values
                    tx_curvature = cam_tx[cam_tx['Curvature'] != 'UNAVAILABLE']
                    if not tx_curvature.empty:
                        try:
                            tx_curvature['Curvature'] = tx_curvature['Curvature'].astype(
                                float)
                            ax8.plot(tx_curvature['rel_time'], tx_curvature['Curvature'],
                                     'g.-', label='TX Curvature')
                        except:
                            print("Warning: Could not convert Curvature to float")

                # Compare with CACC curvature data
                ax8.plot(self.cacc_data['rel_time'], self.cacc_data['leader_curvature'],
                         'b--', label='Leader Curvature (CACC)')

                ax8.set_title('Curvature Comparison')
                ax8.set_xlabel('Time (s)')
                ax8.set_ylabel('Curvature (1/m)')
                ax8.legend()
                ax8.grid(True)

            except Exception as e:
                print(f"Error creating CAM plots: {e}")

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.show()

    def create_vehicle_animation(self):
        """Create animation of vehicle positions"""
        if self.cacc_data is None or len(self.cacc_data) == 0:
            print("No data to visualize")
            return

        # Setup the figure
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.set_title('Vehicle Positions with CAM Messages')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')

        # Get max and min values for x and y to set proper limits
        min_x = min(self.cacc_data['leader_x'].min(),
                    self.cacc_data['follower_x'].min())
        max_x = max(self.cacc_data['leader_x'].max(),
                    self.cacc_data['follower_x'].max())
        min_y = min(self.cacc_data['leader_y'].min(),
                    self.cacc_data['follower_y'].min())
        max_y = max(self.cacc_data['leader_y'].max(),
                    self.cacc_data['follower_y'].max())

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

        # Add CAM message indicators if we have CAM data
        has_cam_data = self.cam_data is not None and len(self.cam_data) > 0
        cam_tx_markers = None
        cam_rx_markers = None

        if has_cam_data:
            # Filter TX messages only
            cam_tx = self.cam_data[self.cam_data['Action'] == 'TX']
            # Create empty scatter plot for CAM TX markers
            cam_tx_markers = ax.scatter([], [], c='g', s=80, marker='*',
                                        alpha=0.7, label='CAM TX')

            # Filter RX messages
            cam_rx = self.cam_data[self.cam_data['Action'] == 'RX']
            # Create empty scatter plot for CAM RX markers
            cam_rx_markers = ax.scatter([], [], c='c', s=80, marker='*',
                                        alpha=0.7, label='CAM RX')

        # Create a status text
        status_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

        ax.grid(True)
        ax.legend(loc='upper right')

        # Downsample data for animation to make it smoother
        step = max(1, len(self.cacc_data) // 300)
        downsampled_data = self.cacc_data.iloc[::step]

        def init():
            leader_path.set_data([], [])
            follower_path.set_data([], [])
            leader_pos.set_data([], [])
            follower_pos.set_data([], [])
            status_text.set_text('')

            if has_cam_data:
                cam_tx_markers.set_offsets(np.empty((0, 2)))
                cam_rx_markers.set_offsets(np.empty((0, 2)))
                return leader_path, follower_path, leader_pos, follower_pos, cam_tx_markers, cam_rx_markers, status_text
            else:
                return leader_path, follower_path, leader_pos, follower_pos, status_text

        def update(frame):
            idx = frame
            current_time = downsampled_data['rel_time'].iloc[idx]

            # Update paths
            leader_path.set_data(
                downsampled_data['leader_x'][:idx], downsampled_data['leader_y'][:idx])
            follower_path.set_data(
                downsampled_data['follower_x'][:idx], downsampled_data['follower_y'][:idx])

            # Update current positions
            leader_x = downsampled_data['leader_x'][idx]
            leader_y = downsampled_data['leader_y'][idx]
            leader_pos.set_data(leader_x, leader_y)

            follower_x = downsampled_data['follower_x'][idx]
            follower_y = downsampled_data['follower_y'][idx]
            follower_pos.set_data(follower_x, follower_y)

            # Update CAM markers if we have CAM data
            cam_status = ""
            if has_cam_data:
                # Show all CAM TX messages up to the current time
                tx_visible = self.cam_data[(self.cam_data['Action'] == 'TX') &
                                           (self.cam_data['rel_time'] <= current_time)]

                if len(tx_visible) > 0:
                    # Get the coordinates for TX messages (leader position when sent)
                    tx_x = []
                    tx_y = []
                    for _, row in tx_visible.iterrows():
                        # Use the nearest leader position from CACC data for each CAM timestamp
                        nearest_idx = (
                            self.cacc_data['rel_time'] - row['rel_time']).abs().idxmin()
                        tx_x.append(
                            self.cacc_data.iloc[nearest_idx]['leader_x'])
                        tx_y.append(
                            self.cacc_data.iloc[nearest_idx]['leader_y'])

                    cam_tx_markers.set_offsets(np.column_stack([tx_x, tx_y]))

                    # Add the most recent TX info to status
                    latest_tx = tx_visible.iloc[-1]
                    cam_status += f"Latest TX: t={latest_tx['rel_time']:.2f}s\n"
                else:
                    cam_tx_markers.set_offsets(np.empty((0, 2)))

                # Show all CAM RX messages up to the current time
                rx_visible = self.cam_data[(self.cam_data['Action'] == 'RX') &
                                           (self.cam_data['rel_time'] <= current_time)]

                if len(rx_visible) > 0:
                    # Get the coordinates for RX messages (follower position when received)
                    rx_x = []
                    rx_y = []
                    for _, row in rx_visible.iterrows():
                        # Use the nearest follower position from CACC data for each CAM timestamp
                        nearest_idx = (
                            self.cacc_data['rel_time'] - row['rel_time']).abs().idxmin()
                        rx_x.append(
                            self.cacc_data.iloc[nearest_idx]['follower_x'])
                        rx_y.append(
                            self.cacc_data.iloc[nearest_idx]['follower_y'])

                    cam_rx_markers.set_offsets(np.column_stack([rx_x, rx_y]))

                    # Add the most recent RX info to status
                    latest_rx = rx_visible.iloc[-1]
                    cam_status += f"Latest RX: t={latest_rx['rel_time']:.2f}s"
                else:
                    cam_rx_markers.set_offsets(np.empty((0, 2)))

            # Update status text
            time_str = f"Time: {current_time:.2f}s"
            vel_str = f"Velocities - L: {downsampled_data['leader_velocity'].iloc[idx]:.2f}, F: {downsampled_data['follower_velocity'].iloc[idx]:.2f} m/s"
            dist_str = f"Distance: {downsampled_data['actual_distance'].iloc[idx]:.2f}m (error: {downsampled_data['distance_error'].iloc[idx]:.2f}m)"

            status_text.set_text(
                f"{time_str}\n{vel_str}\n{dist_str}\n{cam_status}")

            if has_cam_data:
                return leader_path, follower_path, leader_pos, follower_pos, cam_tx_markers, cam_rx_markers, status_text
            else:
                return leader_path, follower_path, leader_pos, follower_pos, status_text

        ani = FuncAnimation(
            fig, update, frames=len(downsampled_data),
            init_func=init, blit=True, interval=50, repeat=True
        )

        plt.tight_layout()
        plt.show()

    def create_cam_timing_analysis(self):
        """Create a focused analysis of CAM message timing and effects"""
        if self.cam_data is None or len(self.cam_data) == 0:
            print("No CAM data available for timing analysis")
            return

        # Create figure for CAM analysis
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('CAM Message Timing Analysis', fontsize=16)
        gs = gridspec.GridSpec(3, 2)

        # 1. CAM message intervals (time between messages)
        ax1 = fig.add_subplot(gs[0, 0])

        # Calculate intervals between consecutive TX messages
        tx_data = self.cam_data[self.cam_data['Action'] == 'TX'].copy()
        if len(tx_data) > 1:
            tx_data['interval'] = tx_data['rel_time'].diff()
            # Filter out the first row which has NaN interval
            tx_intervals = tx_data['interval'].dropna()

            # Plot histogram of intervals
            ax1.hist(tx_intervals, bins=20, alpha=0.7)
            ax1.set_title('CAM TX Message Intervals')
            ax1.set_xlabel('Time between messages (s)')
            ax1.set_ylabel('Frequency')
            ax1.axvline(x=0.1, color='r', linestyle='--',
                        label='Min interval (100ms)')
            ax1.axvline(x=1.0, color='g', linestyle='--',
                        label='Max interval (1s)')
            ax1.legend()
            ax1.grid(True)

        # 2. CAM message generation trigger types
        ax2 = fig.add_subplot(gs[0, 1])

        # Extract trigger information from Details column if available
        if 'Details' in self.cam_data.columns:
            tx_triggers = []
            for _, row in tx_data.iterrows():
                if isinstance(row['Details'], str) and 'Trigger:' in row['Details']:
                    trigger = row['Details'].split('Trigger:')[1].strip()
                    tx_triggers.append(trigger)
                else:
                    tx_triggers.append('unknown')

            # Count trigger types
            trigger_counts = pd.Series(tx_triggers).value_counts()
            ax2.bar(trigger_counts.index, trigger_counts.values)
            ax2.set_title('CAM Generation Triggers')
            ax2.set_xlabel('Trigger Type')
            ax2.set_ylabel('Count')
            plt.setp(ax2.get_xticklabels(), rotation=45, ha='right')
            ax2.grid(True, axis='y')

        # 3. Message latency (if we can calculate it)
        ax3 = fig.add_subplot(gs[1, 0])

        # Try to match TX and RX pairs using StationID and GenDeltaTime
        if 'StationID' in self.cam_data.columns and 'GenDeltaTime' in self.cam_data.columns:
            latencies = []
            times = []

            rx_data = self.cam_data[self.cam_data['Action'] == 'RX'].copy()

            # For each RX message, find matching TX message with same StationID and GenDeltaTime
            for _, rx_row in rx_data.iterrows():
                matching_tx = tx_data[(tx_data['StationID'] == rx_row['StationID']) &
                                      (tx_data['GenDeltaTime'] == rx_row['GenDeltaTime'])]

                if len(matching_tx) == 1:
                    latency = rx_row['rel_time'] - \
                        matching_tx['rel_time'].values[0]
                    if latency >= 0:  # Sanity check
                        latencies.append(latency)
                        times.append(rx_row['rel_time'])

            if latencies:
                ax3.plot(times, latencies, 'b.-')
                ax3.set_title('CAM Message Latency')
                ax3.set_xlabel('Time (s)')
                ax3.set_ylabel('Latency (s)')
                ax3.grid(True)

        # 4. CACC controller response to CAM messages
        ax4 = fig.add_subplot(gs[1, 1])

        # If we have CACC data, try to correlate CAM reception with acceleration changes
        if self.cacc_data is not None:
            # Plot follower acceleration
            ax4.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_accel'],
                     'g-', label='Commanded acceleration')

            # Mark points where CAM messages were received
            rx_times = self.cam_data[self.cam_data['Action']
                                     == 'RX']['rel_time'].values

            for rx_time in rx_times:
                ax4.axvline(x=rx_time, color='r', alpha=0.3, linestyle='--')

            ax4.set_title('CACC Response to CAM Messages')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Acceleration (m/s²)')
            ax4.grid(True)
            # Add legend with custom entry for CAM RX markers
            from matplotlib.lines import Line2D
            custom_lines = [Line2D([0], [0], color='r', lw=2, linestyle='--')]
            ax4.legend(['Commanded acceleration', 'CAM RX'])

        # 5. CAM fields over time
        ax5 = fig.add_subplot(gs[2, 0])

        # Plot CAM speed values from TX messages
        if 'Speed' in self.cam_data.columns:
            speed_data = tx_data[['rel_time', 'Speed']].copy()
            speed_data['Speed'] = pd.to_numeric(
                speed_data['Speed'], errors='coerce')
            ax5.plot(speed_data['rel_time'], speed_data['Speed'],
                     'g.-', label='CAM Speed')

            # If we have CACC data, overlay leader speed for comparison
            if self.cacc_data is not None:
                ax5.plot(self.cacc_data['rel_time'], self.cacc_data['leader_velocity'],
                         'b-', alpha=0.7, label='CACC Leader Speed')

            ax5.set_title('CAM Speed vs CACC Speed')
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Speed (m/s)')
            ax5.legend()
            ax5.grid(True)

        # 6. CAM curvature vs CACC curvature
        ax6 = fig.add_subplot(gs[2, 1])

        # Plot CAM curvature values from TX messages
        if 'Curvature' in self.cam_data.columns:
            # Convert string values to numeric, coercing errors to NaN
            tx_data['Curvature_numeric'] = pd.to_numeric(
                tx_data['Curvature'], errors='coerce')

            # Drop rows with NaN (where conversion failed)
            valid_curvature = tx_data.dropna(subset=['Curvature_numeric'])

            if len(valid_curvature) > 0:
                ax6.plot(valid_curvature['rel_time'], valid_curvature['Curvature_numeric'],
                         'g.-', label='CAM Curvature')

                # If we have CACC data, overlay leader curvature for comparison
                if self.cacc_data is not None:
                    ax6.plot(self.cacc_data['rel_time'], self.cacc_data['leader_curvature'],
                             'b-', alpha=0.7, label='CACC Leader Curvature')

                ax6.set_title('CAM Curvature vs CACC Curvature')
                ax6.set_xlabel('Time (s)')
                ax6.set_ylabel('Curvature (1/m)')
                ax6.legend()
                ax6.grid(True)

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='CACC & CAM Data Visualizer')
    parser.add_argument(
        'cacc_log_file', help='Path to the CACC controller log CSV file')
    parser.add_argument(
        '--cam-log', help='Path to the CAM message log file (required for integrated analysis)')
    parser.add_argument('--static', action='store_true',
                        help='Generate static plots only')
    parser.add_argument('--animation', action='store_true',
                        help='Generate animation plot only')
    parser.add_argument('--cam-analysis', action='store_true',
                        help='Generate detailed CAM timing analysis')

    args = parser.parse_args()

    visualizer = CACCVisualizer(args.cacc_log_file, args.cam_log)

    # Default: show both if no specific option is selected
    show_static = args.static or (not args.animation and not args.cam_analysis)
    show_animation = args.animation or (
        not args.static and not args.cam_analysis)
    show_cam_analysis = args.cam_analysis

    if show_static:
        visualizer.create_static_plots()

    if show_animation:
        visualizer.create_vehicle_animation()

    if show_cam_analysis:
        visualizer.create_cam_timing_analysis()


if __name__ == "__main__":
    main()
