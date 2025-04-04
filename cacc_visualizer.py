#!/usr/bin/env python3

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
import os
import matplotlib.patches as patches
import seaborn as sns
from scipy import stats


class CACCVisualizer:
    def __init__(self, cacc_log_file, cam_log_file=None):
        self.cacc_log_file = cacc_log_file
        self.cam_log_file = cam_log_file
        self.cacc_data = None
        self.cam_data = None

        # Set consistent color scheme for better visual identity
        self.colors = {
            'leader': '#1f77b4',      # blue
            'follower': '#d62728',    # red
            'commanded': '#2ca02c',   # green
            'desired': '#ff7f0e',     # orange
            'background': '#f9f9f9',  # light grey
            'grid': '#e0e0e0',        # medium grey
            'cam_tx': '#9467bd',      # purple
            'cam_rx': '#8c564b',      # brown
            'error_pos': '#c44e52',   # red for position errors
            'error_vel': '#4c72b0',   # blue for velocity errors
        }

        # Set default plot style
        plt.style.use('ggplot')
        plt.rcParams.update({
            'figure.facecolor': self.colors['background'],
            'axes.facecolor': 'white',
            'font.size': 9,
            'axes.titlesize': 11,
            'axes.labelsize': 10,
            'lines.linewidth': 2
        })

        self.load_data()

    def load_data(self):
        """Load data from CSV file(s) with improved error handling and data preprocessing"""
        print(f"Loading CACC data from {self.cacc_log_file}...")
        try:
            self.cacc_data = pd.read_csv(self.cacc_log_file)

            # Convert timestamp to relative time and add more useful time metrics
            if 'timestamp' in self.cacc_data.columns:
                self.cacc_data['rel_time'] = self.cacc_data['timestamp'] - \
                    self.cacc_data['timestamp'].iloc[0]
                # Add minutes for longer runs
                self.cacc_data['time_min'] = self.cacc_data['rel_time'] / 60

            # Calculate additional metrics for analysis
            if 'distance_error' in self.cacc_data.columns:
                # Calculate rolling statistics for distance error
                # Adaptive window based on data size
                window_size = min(20, len(self.cacc_data) // 10)
                self.cacc_data['error_ma'] = self.cacc_data['distance_error'].rolling(
                    window=window_size, center=True).mean()
                self.cacc_data['error_std'] = self.cacc_data['distance_error'].rolling(
                    window=window_size, center=True).std()

                # Calculate RMSE for distance error
                self.distance_error_rmse = np.sqrt(
                    np.mean(self.cacc_data['distance_error']**2))

            # Add acceleration derivatives for jerk analysis if needed
            if 'follower_acceleration' in self.cacc_data.columns and len(self.cacc_data) > 2:
                # Calculate time differences for proper derivatives
                self.cacc_data['dt'] = self.cacc_data['rel_time'].diff()
                # Replace zero dt with small value to avoid division by zero
                self.cacc_data['dt'].replace(0, np.nan, inplace=True)
                self.cacc_data['dt'].fillna(
                    self.cacc_data['dt'].mean(), inplace=True)

                # Calculate jerk (rate of change of acceleration)
                self.cacc_data['jerk'] = self.cacc_data['follower_acceleration'].diff(
                ) / self.cacc_data['dt']
                # Remove extreme values (could be due to measurement noise)
                jerk_std = self.cacc_data['jerk'].std()
                self.cacc_data['jerk'] = self.cacc_data['jerk'].clip(
                    -3*jerk_std, 3*jerk_std)

            print(f"Loaded {len(self.cacc_data)} CACC data points")

            # Calculate basic stats for key metrics
            print("\nCACC Data Summary Statistics:")
            key_metrics = ['leader_velocity', 'follower_velocity',
                           'actual_distance', 'distance_error']
            for metric in key_metrics:
                if metric in self.cacc_data.columns:
                    mean = self.cacc_data[metric].mean()
                    std = self.cacc_data[metric].std()
                    min_val = self.cacc_data[metric].min()
                    max_val = self.cacc_data[metric].max()
                    print(
                        f"  {metric}: mean={mean:.2f}, std={std:.2f}, min={min_val:.2f}, max={max_val:.2f}")

            # Load CAM data if file path is provided
            if self.cam_log_file and os.path.exists(self.cam_log_file):
                print(f"Loading CAM data from {self.cam_log_file}...")
                try:
                    # Skip the first line (header description) if it's a comment
                    with open(self.cam_log_file, 'r') as f:
                        first_line = f.readline()
                        if '===' in first_line:  # This is our header comment
                            self.cam_data = pd.read_csv(
                                self.cam_log_file, skiprows=1)
                        else:  # No header comment, read normally
                            f.seek(0)
                            self.cam_data = pd.read_csv(self.cam_log_file)

                    # Convert timestamp to relative time aligned with CACC data
                    if 'Timestamp' in self.cam_data.columns:
                        # Align with CACC data start time if possible
                        start_time = self.cacc_data['timestamp'].iloc[
                            0] if 'timestamp' in self.cacc_data.columns else self.cam_data['Timestamp'].iloc[0]
                        self.cam_data['rel_time'] = self.cam_data['Timestamp'] - start_time

                    # Calculate some CAM metrics
                    if 'Action' in self.cam_data.columns:
                        tx_data = self.cam_data[self.cam_data['Action'] == 'TX']
                        rx_data = self.cam_data[self.cam_data['Action'] == 'RX']

                        if len(tx_data) > 1:
                            tx_data['interval'] = tx_data['rel_time'].diff().dropna()
                            self.tx_interval_mean = tx_data['interval'].mean()
                            self.tx_interval_std = tx_data['interval'].std()
                            print(
                                f"  CAM TX: count={len(tx_data)}, interval={self.tx_interval_mean:.3f}±{self.tx_interval_std:.3f}s")

                        if len(rx_data) > 1:
                            rx_data['interval'] = rx_data['rel_time'].diff().dropna()
                            self.rx_interval_mean = rx_data['interval'].mean()
                            self.rx_interval_std = rx_data['interval'].std()
                            print(
                                f"  CAM RX: count={len(rx_data)}, interval={self.rx_interval_mean:.3f}±{self.rx_interval_std:.3f}s")

                except Exception as e:
                    print(f"Error loading CAM data: {e}")
                    self.cam_data = None
            elif self.cam_log_file:
                print(f"CAM log file not found: {self.cam_log_file}")

        except Exception as e:
            print(f"Error loading CACC data: {e}")
            exit(1)

    def create_static_plots(self):
        """Create focused static plots showing only the most relevant metrics"""
        if self.cacc_data is None or len(self.cacc_data) == 0:
            print("No CACC data to visualize")
            return

        # Create a figure with optimal layout
        fig = plt.figure(figsize=(14, 16))
        gs = gridspec.GridSpec(5, 2, height_ratios=[1, 1, 1, 1, 1.2])

        # Main title with file information
        fig.suptitle('CACC Controller Performance Analysis',
                     fontsize=14, fontweight='bold')

        # 1. Velocities plot (top left)
        ax_vel = fig.add_subplot(gs[0, 0])
        ax_vel.plot(self.cacc_data['rel_time'], self.cacc_data['leader_velocity'],
                    color=self.colors['leader'], label='Leader')
        ax_vel.plot(self.cacc_data['rel_time'], self.cacc_data['follower_velocity'],
                    color=self.colors['follower'], label='Follower')
        ax_vel.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_velocity'],
                    color=self.colors['commanded'], linestyle='--', label='Commanded')

        # Add statistical annotations
        leader_mean = self.cacc_data['leader_velocity'].mean()
        follower_mean = self.cacc_data['follower_velocity'].mean()
        ax_vel.axhline(
            y=leader_mean, color=self.colors['leader'], linestyle=':', alpha=0.7)
        ax_vel.axhline(y=follower_mean,
                       color=self.colors['follower'], linestyle=':', alpha=0.7)

        # Add velocity stats text
        vel_stats = f"Avg: Leader={leader_mean:.2f}, Follower={follower_mean:.2f} m/s\n" \
                    f"Max: Leader={self.cacc_data['leader_velocity'].max():.2f}, " \
                    f"Follower={self.cacc_data['follower_velocity'].max():.2f} m/s"
        ax_vel.text(0.02, 0.02, vel_stats, transform=ax_vel.transAxes, fontsize=8,
                    bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_vel.set_title('Vehicle Velocities')
        ax_vel.set_ylabel('Velocity (m/s)')
        ax_vel.legend(loc='upper right')
        ax_vel.grid(True, alpha=0.3)

        # 2. Distance and errors (top right)
        ax_dist = fig.add_subplot(gs[0, 1])
        ax_dist.plot(self.cacc_data['rel_time'], self.cacc_data['desired_distance'],
                     color=self.colors['desired'], linestyle='--', label='Desired')
        ax_dist.plot(self.cacc_data['rel_time'], self.cacc_data['actual_distance'],
                     color=self.colors['follower'], label='Actual')

        # Add error regions
        ax_dist.fill_between(self.cacc_data['rel_time'],
                             self.cacc_data['desired_distance'],
                             self.cacc_data['actual_distance'],
                             where=self.cacc_data['actual_distance'] > self.cacc_data['desired_distance'],
                             color='red', alpha=0.2, label='Too Far')
        ax_dist.fill_between(self.cacc_data['rel_time'],
                             self.cacc_data['desired_distance'],
                             self.cacc_data['actual_distance'],
                             where=self.cacc_data['actual_distance'] < self.cacc_data['desired_distance'],
                             color='blue', alpha=0.2, label='Too Close')

        # Add RMSE annotation
        ax_dist.text(0.02, 0.02, f"RMSE: {self.distance_error_rmse:.3f} m",
                     transform=ax_dist.transAxes, fontsize=9, fontweight='bold',
                     bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_dist.set_title('Following Distance')
        ax_dist.set_ylabel('Distance (m)')
        ax_dist.legend(loc='upper right')
        ax_dist.grid(True, alpha=0.3)

        # 3. Combined error state plot (row 2, left)
        ax_err = fig.add_subplot(gs[1, 0])
        # z1, z2 are position errors (longitudinal, lateral)
        ax_err.plot(self.cacc_data['rel_time'], self.cacc_data['z1'],
                    color=self.colors['error_pos'], label='z₁: Long. Position Error')
        ax_err.plot(self.cacc_data['rel_time'], self.cacc_data['z2'],
                    color=self.colors['error_vel'], label='z₂: Lat. Position Error')

        # Add zero line and shade error regions
        ax_err.axhline(y=0, color='black', linestyle='-', linewidth=0.5)

        # Get max absolute error for z1, z2 for symmetric y-axis
        max_error = max(abs(self.cacc_data['z1'].max()), abs(self.cacc_data['z1'].min()),
                        abs(self.cacc_data['z2'].max()), abs(self.cacc_data['z2'].min()))
        ax_err.set_ylim(-max_error*1.1, max_error*1.1)

        # Calculate RMS errors
        z1_rmse = np.sqrt(np.mean(self.cacc_data['z1']**2))
        z2_rmse = np.sqrt(np.mean(self.cacc_data['z2']**2))
        error_stats = f"Long. RMSE: {z1_rmse:.3f} m\nLat. RMSE: {z2_rmse:.3f} m"
        ax_err.text(0.02, 0.95, error_stats, transform=ax_err.transAxes, fontsize=8,
                    va='top', bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_err.set_title('Position Error States')
        ax_err.set_ylabel('Error (m)')
        ax_err.legend(loc='upper right')
        ax_err.grid(True, alpha=0.3)

        # 4. Acceleration and control inputs (row 2, right)
        ax_acc = fig.add_subplot(gs[1, 1])
        ax_acc.plot(self.cacc_data['rel_time'], self.cacc_data['leader_acceleration'],
                    color=self.colors['leader'], label='Leader Accel.')
        ax_acc.plot(self.cacc_data['rel_time'], self.cacc_data['follower_acceleration'],
                    color=self.colors['follower'], label='Follower Accel.')
        ax_acc.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_accel'],
                    color=self.colors['commanded'], linestyle='--', label='Cmd Accel.')

        # Add zero line
        ax_acc.axhline(y=0, color='black', linestyle='-', linewidth=0.5)

        # Add acceleration stats
        accel_stats = (f"Max decel: {self.cacc_data['follower_acceleration'].min():.2f} m/s²\n"
                       f"Max accel: {self.cacc_data['follower_acceleration'].max():.2f} m/s²")
        ax_acc.text(0.02, 0.02, accel_stats, transform=ax_acc.transAxes, fontsize=8,
                    bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_acc.set_title('Vehicle Accelerations')
        ax_acc.set_ylabel('Acceleration (m/s²)')
        ax_acc.legend(loc='upper right')
        ax_acc.grid(True, alpha=0.3)

        # 5. Velocity errors and control (row 3, left)
        ax_vel_err = fig.add_subplot(gs[2, 0])
        # z3, z4 are velocity errors (longitudinal, lateral)
        ax_vel_err.plot(self.cacc_data['rel_time'], self.cacc_data['z3'],
                        color=self.colors['error_pos'], label='z₃: Long. Velocity Error')
        ax_vel_err.plot(self.cacc_data['rel_time'], self.cacc_data['z4'],
                        color=self.colors['error_vel'], label='z₄: Lat. Velocity Error')

        # Add zero line
        ax_vel_err.axhline(y=0, color='black', linestyle='-', linewidth=0.5)

        # Calculate RMS errors
        z3_rmse = np.sqrt(np.mean(self.cacc_data['z3']**2))
        z4_rmse = np.sqrt(np.mean(self.cacc_data['z4']**2))
        vel_error_stats = f"Long. RMSE: {z3_rmse:.3f} m/s\nLat. RMSE: {z4_rmse:.3f} m/s"
        ax_vel_err.text(0.02, 0.95, vel_error_stats, transform=ax_vel_err.transAxes, fontsize=8,
                        va='top', bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_vel_err.set_title('Velocity Error States')
        ax_vel_err.set_ylabel('Error (m/s)')
        ax_vel_err.legend(loc='upper right')
        ax_vel_err.grid(True, alpha=0.3)

        # 6. Angular control and curvature (row 3, right)
        ax_ang = fig.add_subplot(gs[2, 1])
        ax_ang.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_angular'],
                    color=self.colors['commanded'], label='Cmd Angular')

        # Add curvature on secondary axis if available
        ax_curv = ax_ang.twinx()
        if 'leader_curvature' in self.cacc_data.columns:
            ax_curv.plot(self.cacc_data['rel_time'], self.cacc_data['leader_curvature'],
                         color='purple', alpha=0.7, linestyle='--', label='Leader Curvature')
            ax_curv.set_ylabel('Curvature (1/m)', color='purple')

        ax_ang.set_title('Angular Control & Curvature')
        ax_ang.set_ylabel('Angular Velocity (rad/s)')

        # Add legends for both axes
        lines1, labels1 = ax_ang.get_legend_handles_labels()
        lines2, labels2 = ax_curv.get_legend_handles_labels()
        ax_ang.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        ax_ang.grid(True, alpha=0.3)

        # 7. Path visualization (row 4)
        ax_path = fig.add_subplot(gs[3, :])
        ax_path.plot(self.cacc_data['leader_x'], self.cacc_data['leader_y'],
                     color=self.colors['leader'], label='Leader Path')
        ax_path.plot(self.cacc_data['follower_x'], self.cacc_data['follower_y'],
                     color=self.colors['follower'], label='Follower Path')

        # Mark start and end positions
        ax_path.scatter(self.cacc_data['leader_x'].iloc[0], self.cacc_data['leader_y'].iloc[0],
                        color='green', s=100, marker='o', label='Start')
        ax_path.scatter(self.cacc_data['leader_x'].iloc[-1], self.cacc_data['leader_y'].iloc[-1],
                        color='red', s=100, marker='x', label='End')

        # Add trajectory distance info
        leader_distance = np.sum(np.sqrt(np.diff(self.cacc_data['leader_x'])**2 +
                                         np.diff(self.cacc_data['leader_y'])**2))
        follower_distance = np.sum(np.sqrt(np.diff(self.cacc_data['follower_x'])**2 +
                                           np.diff(self.cacc_data['follower_y'])**2))

        path_info = f"Leader path: {leader_distance:.1f}m\nFollower path: {follower_distance:.1f}m"
        ax_path.text(0.02, 0.02, path_info, transform=ax_path.transAxes,
                     bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        ax_path.set_title('Vehicle Trajectories')
        ax_path.set_xlabel('X Position (m)')
        ax_path.set_ylabel('Y Position (m)')
        ax_path.legend(loc='upper right')
        ax_path.grid(True, alpha=0.3)
        ax_path.set_aspect('equal')

        # 8. CAM message analysis (if available) or Controller input analysis (row 5, full width)
        if self.cam_data is not None and len(self.cam_data) > 0:
            # CAM Message visualization combined with controller response
            ax_cam = fig.add_subplot(gs[4, :])

            # Plot follower acceleration
            ax_cam.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_accel'],
                        color=self.colors['commanded'], label='Cmd Accel')

            # Mark CAM message events
            tx_data = self.cam_data[self.cam_data['Action'] == 'TX']
            rx_data = self.cam_data[self.cam_data['Action'] == 'RX']

            # Create markers at the bottom of the plot for CAM events
            for tx_time in tx_data['rel_time']:
                ax_cam.axvline(
                    x=tx_time, color=self.colors['cam_tx'], alpha=0.5, linestyle='--', lw=0.8)

            for rx_time in rx_data['rel_time']:
                ax_cam.axvline(
                    x=rx_time, color=self.colors['cam_rx'], alpha=0.5, linestyle='--', lw=0.8)

            # Add CAM message stats
            if len(tx_data) > 1 and len(rx_data) > 1:
                cam_stats = (f"TX: {len(tx_data)} msgs, {self.tx_interval_mean:.3f}±{self.tx_interval_std:.3f}s\n"
                             f"RX: {len(rx_data)} msgs, {self.rx_interval_mean:.3f}±{self.rx_interval_std:.3f}s")
                ax_cam.text(0.02, 0.95, cam_stats, transform=ax_cam.transAxes, va='top',
                            bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

            # Create legend with proper entries
            from matplotlib.lines import Line2D
            custom_lines = [
                Line2D([0], [0], color=self.colors['commanded'], lw=2),
                Line2D([0], [0], color=self.colors['cam_tx'], linestyle='--'),
                Line2D([0], [0], color=self.colors['cam_rx'], linestyle='--')
            ]
            ax_cam.legend(
                custom_lines, ['Commanded Accel', 'CAM TX', 'CAM RX'], loc='upper right')

            ax_cam.set_title('CAM Messages & Controller Response')
            ax_cam.set_xlabel('Time (s)')
            ax_cam.set_ylabel('Acceleration (m/s²)')
            ax_cam.grid(True, alpha=0.3)
        else:
            # Controller input analysis - show contributions to control decisions
            ax_ctrl = fig.add_subplot(gs[4, :])

            if all(col in self.cacc_data.columns for col in
                   ['inP1_pos_err', 'inP1_vel_err', 'inP1_geom_vel', 'inP1_yaw_rate']):
                # Plot contribution components
                ax_ctrl.plot(self.cacc_data['rel_time'], self.cacc_data['inP1_pos_err'],
                             label='Position Error', color='#e377c2')
                ax_ctrl.plot(self.cacc_data['rel_time'], self.cacc_data['inP1_vel_err'],
                             label='Velocity Error', color='#7f7f7f')
                ax_ctrl.plot(self.cacc_data['rel_time'], self.cacc_data['inP1_geom_vel'],
                             label='Geometric Velocity', color='#bcbd22')
                ax_ctrl.plot(self.cacc_data['rel_time'], self.cacc_data['inP1_yaw_rate'],
                             label='Yaw Rate', color='#17becf')

                # Add the sum (inP_1) and commanded acceleration for comparison
                ax_ctrl.plot(self.cacc_data['rel_time'], self.cacc_data['commanded_accel'],
                             color=self.colors['commanded'], linestyle='--', lw=3,
                             label='Commanded Accel')

                ax_ctrl.set_title('Control Input Components')
                ax_ctrl.set_xlabel('Time (s)')
                ax_ctrl.set_ylabel('Contribution')
                ax_ctrl.legend(loc='upper right')
                ax_ctrl.grid(True, alpha=0.3)
            else:
                # If control components are not available, show a correlation heatmap
                corr_cols = ['leader_velocity', 'follower_velocity', 'actual_distance',
                             'distance_error', 'z1', 'z2', 'z3', 'z4', 'commanded_accel',
                             'commanded_angular']

                # Only include columns that actually exist
                corr_cols = [
                    col for col in corr_cols if col in self.cacc_data.columns]

                # Calculate correlation matrix
                corr_matrix = self.cacc_data[corr_cols].corr()

                # Create heatmap
                sns.heatmap(corr_matrix, annot=True, fmt=".2f", cmap='coolwarm',
                            ax=ax_ctrl, vmin=-1, vmax=1)

                ax_ctrl.set_title('Correlation Matrix of Key CACC Variables')

        # Set common x-labels for time-based plots
        for ax in [ax_vel, ax_dist, ax_err, ax_acc, ax_vel_err, ax_ang]:
            ax.set_xlabel('Time (s)')

        plt.tight_layout()
        plt.subplots_adjust(top=0.95, hspace=0.3, wspace=0.25)

        # Save plot
        save_path = os.path.splitext(self.cacc_log_file)[0] + "_analysis.png"
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved analysis plot to {save_path}")

        plt.show()

    def create_vehicle_animation(self):
        """Create enhanced animation with more informative visuals"""
        if self.cacc_data is None or len(self.cacc_data) == 0:
            print("No data to visualize")
            return

        # Create figure with subplot grid for animation plus telemetry
        fig = plt.figure(figsize=(12, 9))
        gs = gridspec.GridSpec(3, 2, height_ratios=[3, 1, 1])

        # Main plot for vehicle positions
        ax_main = fig.add_subplot(gs[0, :])
        ax_main.set_title('Vehicle Positions with CACC Control')
        ax_main.set_xlabel('X Position (m)')
        ax_main.set_ylabel('Y Position (m)')

        # Bottom plots for telemetry
        ax_vel = fig.add_subplot(gs[1, :])  # For velocities
        ax_dist = fig.add_subplot(gs[2, :])  # For distances

        # Set background colors
        fig.patch.set_facecolor(self.colors['background'])
        ax_main.set_facecolor('white')
        ax_vel.set_facecolor('white')
        ax_dist.set_facecolor('white')

        # Get min/max values for plot limits with margin
        min_x = min(self.cacc_data['leader_x'].min(),
                    self.cacc_data['follower_x'].min())
        max_x = max(self.cacc_data['leader_x'].max(),
                    self.cacc_data['follower_x'].max())
        min_y = min(self.cacc_data['leader_y'].min(),
                    self.cacc_data['follower_y'].min())
        max_y = max(self.cacc_data['leader_y'].max(),
                    self.cacc_data['follower_y'].max())

        # Add margin
        margin = max(max_x - min_x, max_y - min_y) * 0.1
        ax_main.set_xlim(min_x - margin, max_x + margin)
        ax_main.set_ylim(min_y - margin, max_y + margin)
        ax_main.set_aspect('equal')

        # Create empty objects for animation
        leader_path, = ax_main.plot([], [], '-', color=self.colors['leader'], alpha=0.5,
                                    linewidth=2, label='Leader Path')
        follower_path, = ax_main.plot([], [], '-', color=self.colors['follower'], alpha=0.5,
                                      linewidth=2, label='Follower Path')

        # Use actual vehicle shapes instead of simple dots
        vehicle_length = 0.6
        vehicle_width = 0.3

        leader_vehicle = patches.Rectangle((0, 0), vehicle_length, vehicle_width,
                                           color=self.colors['leader'],
                                           alpha=0.8, angle=0)
        follower_vehicle = patches.Rectangle((0, 0), vehicle_length, vehicle_width,
                                             color=self.colors['follower'],
                                             alpha=0.8, angle=0)

        ax_main.add_patch(leader_vehicle)
        ax_main.add_patch(follower_vehicle)

        # Line connecting vehicles
        connect_line, = ax_main.plot([], [], 'k--', alpha=0.7, linewidth=1.5)

        # Status text display
        status_text = ax_main.text(0.02, 0.98, '', transform=ax_main.transAxes,
                                   va='top', ha='left', fontsize=9,
                                   bbox=dict(facecolor='white', alpha=0.7))

        # Create empty plots for velocity subplot
        leader_vel_line, = ax_vel.plot([], [], '-', color=self.colors['leader'],
                                       label='Leader')
        follower_vel_line, = ax_vel.plot([], [], '-', color=self.colors['follower'],
                                         label='Follower')
        commanded_vel_line, = ax_vel.plot([], [], '--', color=self.colors['commanded'],
                                          label='Commanded')
        current_time_vel = ax_vel.axvline(
            x=0, color='k', linestyle='-', alpha=0.7)

        ax_vel.set_title('Vehicle Velocities')
        ax_vel.set_xlabel('Time (s)')
        ax_vel.set_ylabel('Velocity (m/s)')
        ax_vel.grid(True, alpha=0.3)
        ax_vel.legend(loc='upper right')

        # Create empty plots for distance subplot
        actual_dist_line, = ax_dist.plot([], [], '-', color=self.colors['follower'],
                                         label='Actual')
        desired_dist_line, = ax_dist.plot([], [], '--', color=self.colors['desired'],
                                          label='Desired')
        current_time_dist = ax_dist.axvline(
            x=0, color='k', linestyle='-', alpha=0.7)

        ax_dist.set_title('Following Distance')
        ax_dist.set_xlabel('Time (s)')
        ax_dist.set_ylabel('Distance (m)')
        ax_dist.grid(True, alpha=0.3)
        ax_dist.legend(loc='upper right')

        # Set y-axis limits for velocity and distance plots based on data
        ax_vel.set_ylim(
            min(self.cacc_data['leader_velocity'].min(),
                self.cacc_data['follower_velocity'].min()) - 0.5,
            max(self.cacc_data['leader_velocity'].max(),
                self.cacc_data['follower_velocity'].max()) + 0.5
        )

        ax_dist.set_ylim(
            min(self.cacc_data['desired_distance'].min(),
                self.cacc_data['actual_distance'].min()) - 0.5,
            max(self.cacc_data['desired_distance'].max(),
                self.cacc_data['actual_distance'].max()) + 0.5
        )

        # Set x-axis limits for both plots
        ax_vel.set_xlim(0, self.cacc_data['rel_time'].max())
        ax_dist.set_xlim(0, self.cacc_data['rel_time'].max())

        # Time window for velocity and distance plots
        # 30 seconds or max time if shorter
        time_window = min(30.0, self.cacc_data['rel_time'].max())

        # Add CAM message markers if available
        has_cam_data = self.cam_data is not None and len(self.cam_data) > 0
        cam_tx_markers = None
        cam_rx_markers = None

        if has_cam_data:
            cam_tx_markers = ax_main.scatter([], [], color=self.colors['cam_tx'],
                                             s=80, marker='*', alpha=0.8, label='CAM TX')
            cam_rx_markers = ax_main.scatter([], [], color=self.colors['cam_rx'],
                                             s=80, marker='*', alpha=0.8, label='CAM RX')

        # Initialize the animation
        def init():
            leader_path.set_data([], [])
            follower_path.set_data([], [])
            connect_line.set_data([], [])
            status_text.set_text('')

            leader_vehicle.set_xy([0, 0])
            follower_vehicle.set_xy([0, 0])

            leader_vel_line.set_data([], [])
            follower_vel_line.set_data([], [])
            commanded_vel_line.set_data([], [])
            current_time_vel.set_xdata([0])

            actual_dist_line.set_data([], [])
            desired_dist_line.set_data([], [])
            current_time_dist.set_xdata([0])

            if has_cam_data:
                cam_tx_markers.set_offsets(np.empty((0, 2)))
                cam_rx_markers.set_offsets(np.empty((0, 2)))
                return (leader_path, follower_path, leader_vehicle, follower_vehicle,
                        connect_line, status_text, leader_vel_line, follower_vel_line,
                        commanded_vel_line, current_time_vel, actual_dist_line, desired_dist_line,
                        current_time_dist, cam_tx_markers, cam_rx_markers)
            else:
                return (leader_path, follower_path, leader_vehicle, follower_vehicle,
                        connect_line, status_text, leader_vel_line, follower_vel_line,
                        commanded_vel_line, current_time_vel, actual_dist_line, desired_dist_line,
                        current_time_dist)

        # Update function for animation
        def update(frame):
            # Get current data
            idx = frame
            current_time = self.cacc_data['rel_time'].iloc[idx]

            # Update paths
            leader_path.set_data(
                self.cacc_data['leader_x'][:idx+1],
                self.cacc_data['leader_y'][:idx+1]
            )
            follower_path.set_data(
                self.cacc_data['follower_x'][:idx+1],
                self.cacc_data['follower_y'][:idx+1]
            )

            # Update vehicle positions and orientations
            lx = self.cacc_data['leader_x'].iloc[idx]
            ly = self.cacc_data['leader_y'].iloc[idx]
            fx = self.cacc_data['follower_x'].iloc[idx]
            fy = self.cacc_data['follower_y'].iloc[idx]

            # Calculate orientation from positions or use yaw if available
            if idx > 0:
                ldx = self.cacc_data['leader_x'].iloc[idx] - \
                    self.cacc_data['leader_x'].iloc[idx-1]
                ldy = self.cacc_data['leader_y'].iloc[idx] - \
                    self.cacc_data['leader_y'].iloc[idx-1]
                leader_angle = np.degrees(np.arctan2(ldy, ldx)) if (
                    ldx != 0 or ldy != 0) else 0

                fdx = self.cacc_data['follower_x'].iloc[idx] - \
                    self.cacc_data['follower_x'].iloc[idx-1]
                fdy = self.cacc_data['follower_y'].iloc[idx] - \
                    self.cacc_data['follower_y'].iloc[idx-1]
                follower_angle = np.degrees(np.arctan2(fdy, fdx)) if (
                    fdx != 0 or fdy != 0) else 0
            else:
                leader_angle = 0
                follower_angle = 0

            # Use pose_yaw and cam_yaw if available
            if 'pose_yaw' in self.cacc_data.columns and 'cam_yaw' in self.cacc_data.columns:
                leader_angle = np.degrees(self.cacc_data['cam_yaw'].iloc[idx])
                follower_angle = np.degrees(
                    self.cacc_data['pose_yaw'].iloc[idx])

            # Calculate corners with orientation
            leader_vehicle._angle = leader_angle
            leader_vehicle.set_xy(
                [lx - vehicle_length/2, ly - vehicle_width/2])

            follower_vehicle._angle = follower_angle
            follower_vehicle.set_xy(
                [fx - vehicle_length/2, fy - vehicle_width/2])

            # Update connection line between vehicles
            connect_line.set_data([lx, fx], [ly, fy])

            # Update velocity subplot
            time_array = self.cacc_data['rel_time'].values
            # Find indices for the time window
            start_idx = max(0, np.searchsorted(
                time_array, current_time - time_window))

            # Update velocity lines
            leader_vel_line.set_data(
                time_array[start_idx:idx+1],
                self.cacc_data['leader_velocity'].iloc[start_idx:idx+1]
            )
            follower_vel_line.set_data(
                time_array[start_idx:idx+1],
                self.cacc_data['follower_velocity'].iloc[start_idx:idx+1]
            )
            commanded_vel_line.set_data(
                time_array[start_idx:idx+1],
                self.cacc_data['commanded_velocity'].iloc[start_idx:idx+1]
            )

            # Update time marker
            current_time_vel.set_xdata([current_time])

            # Set visible window for velocity plot
            if current_time > time_window:
                ax_vel.set_xlim(current_time - time_window, current_time)

            # Update distance subplot
            actual_dist_line.set_data(
                time_array[start_idx:idx+1],
                self.cacc_data['actual_distance'].iloc[start_idx:idx+1]
            )
            desired_dist_line.set_data(
                time_array[start_idx:idx+1],
                self.cacc_data['desired_distance'].iloc[start_idx+1]
            )

            # Update time marker
            current_time_dist.set_xdata([current_time])

            # Set visible window for distance plot
            if current_time > time_window:
                ax_dist.set_xlim(current_time - time_window, current_time)

            # Update status text with current metrics
            status = (
                f"Time: {current_time:.2f}s\n"
                f"Velocities: L={self.cacc_data['leader_velocity'].iloc[idx]:.2f}, "
                f"F={self.cacc_data['follower_velocity'].iloc[idx]:.2f} m/s\n"
                f"Distance: {self.cacc_data['actual_distance'].iloc[idx]:.2f}m "
                f"(desired: {self.cacc_data['desired_distance'].iloc[idx]:.2f}m)\n"
                f"Error: {self.cacc_data['distance_error'].iloc[idx]:.2f}m"
            )
            status_text.set_text(status)

            # Update CAM message markers if available
            if has_cam_data:
                # Show CAM messages for current visible time window
                if current_time > time_window:
                    window_start = current_time - time_window
                else:
                    window_start = 0

                # Filter TX messages in the visible window
                visible_tx = self.cam_data[(self.cam_data['Action'] == 'TX') &
                                           (self.cam_data['rel_time'] <= current_time) &
                                           (self.cam_data['rel_time'] >= window_start)]

                if len(visible_tx) > 0:
                    # Use the leader position at the message time
                    tx_pos = []
                    for _, row in visible_tx.iterrows():
                        # Find nearest CACC data point
                        nearest_idx = np.argmin(
                            np.abs(self.cacc_data['rel_time'] - row['rel_time']))
                        tx_pos.append([
                            self.cacc_data['leader_x'].iloc[nearest_idx],
                            self.cacc_data['leader_y'].iloc[nearest_idx]
                        ])

                    cam_tx_markers.set_offsets(tx_pos)
                else:
                    cam_tx_markers.set_offsets(np.empty((0, 2)))

                # Filter RX messages in the visible window
                visible_rx = self.cam_data[(self.cam_data['Action'] == 'RX') &
                                           (self.cam_data['rel_time'] <= current_time) &
                                           (self.cam_data['rel_time'] >= window_start)]

                if len(visible_rx) > 0:
                    # Use the follower position at the message time
                    rx_pos = []
                    for _, row in visible_rx.iterrows():
                        # Find nearest CACC data point
                        nearest_idx = np.argmin(
                            np.abs(self.cacc_data['rel_time'] - row['rel_time']))
                        rx_pos.append([
                            self.cacc_data['follower_x'].iloc[nearest_idx],
                            self.cacc_data['follower_y'].iloc[nearest_idx]
                        ])

                    cam_rx_markers.set_offsets(rx_pos)
                else:
                    cam_rx_markers.set_offsets(np.empty((0, 2)))

                return (leader_path, follower_path, leader_vehicle, follower_vehicle,
                        connect_line, status_text, leader_vel_line, follower_vel_line,
                        commanded_vel_line, current_time_vel, actual_dist_line, desired_dist_line,
                        current_time_dist, cam_tx_markers, cam_rx_markers)
            else:
                return (leader_path, follower_path, leader_vehicle, follower_vehicle,
                        connect_line, status_text, leader_vel_line, follower_vel_line,
                        commanded_vel_line, current_time_vel, actual_dist_line, desired_dist_line,
                        current_time_dist)

        # Create animation with down-sampling for smoother playback
        # Determine appropriate sampling rate
        frame_count = len(self.cacc_data)
        max_frames = 500  # Limit to reasonable number of frames
        step = max(1, frame_count // max_frames)

        frame_indices = range(0, frame_count, step)

        ani = FuncAnimation(
            fig, update, frames=frame_indices,
            init_func=init, blit=True, interval=50, repeat=True
        )

        # Add legend
        ax_main.legend(loc='upper right')

        # Add play/pause functionality with keyboard shortcuts
        plt.tight_layout()

        # Show plot
        print("Animation controls: Space=Play/Pause, Left/Right=Step, Home=Restart")
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Enhanced CACC & CAM Data Visualizer')
    parser.add_argument(
        'cacc_log_file', help='Path to the CACC controller log CSV file')
    parser.add_argument('--cam-log', help='Path to the CAM message log file')
    parser.add_argument('--static', action='store_true',
                        help='Generate static plots only')
    parser.add_argument('--animation', action='store_true',
                        help='Generate animation only')

    args = parser.parse_args()

    visualizer = CACCVisualizer(args.cacc_log_file, args.cam_log)

    # Default: show both if no specific option is selected
    show_static = args.static or (not args.animation)
    show_animation = args.animation or (not args.static)

    if show_static:
        visualizer.create_static_plots()

    if show_animation:
        visualizer.create_vehicle_animation()


if __name__ == "__main__":
    main()
