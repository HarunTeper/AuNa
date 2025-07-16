#!/usr/bin/env python3

"""Curve Fitting Node"""
import csv
import numpy as np
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from scipy.interpolate import griddata


class CurveFitting(Node):
    """Curve Fitting Node"""

    def __init__(self):
        """Initializes the node"""
        super().__init__('waypoint_publisher')
        self.declare_parameter('waypoint_file', 'waypoints.csv')
        self.declare_parameter('interpolation_distance', 0.1)
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('plot_results', False)

        self.interpolated_waypoints = []
        self.read_csv()
        self.interpolate_waypoints()
        self.save_interpolated_waypoints()
        self.print_angles_between_waypoints()
        self.plot_interpolated_waypoints()

    def read_csv(self):
        """Reads the waypoints from the csv file and swaps if swap_xy is true"""
        self.waypoints = []
        with open(self.get_parameter('waypoint_file').value, 'r', encoding='utf-8-sig') as file:
            reader = csv.reader(file)
            for row in reader:
                if self.get_parameter('swap_xy').value:
                    self.waypoints.append((float(row[1]), float(row[0])))
                else:
                    self.waypoints.append((float(row[0]), float(row[1])))

    def interpolate_waypoints(self):
        """Interpolates the waypoints based on the distance between them"""
        waypoints = np.array(self.waypoints)
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        t = np.arange(len(waypoints))

        # Fine-grained sampling of waypoints
        num_points = len(self.waypoints) * 1000  # Increase the number of points for fine-grained sampling
        t_fine = np.linspace(0, len(waypoints) - 1, num_points)
        x_fine = np.interp(t_fine, t, x)
        y_fine = np.interp(t_fine, t, y)

        # Filter waypoints based on distance
        new_waypoints = list(zip(x_fine, y_fine))

        filtered_waypoints = [new_waypoints[0]]
        accumulated_distance = 0

        for i in range(len(new_waypoints) - 1):
            segment_distance = np.sqrt((new_waypoints[i+1][0] - new_waypoints[i][0]) ** 2 +
                                    (new_waypoints[i+1][1] - new_waypoints[i][1]) ** 2)
            accumulated_distance += segment_distance

            if accumulated_distance >= self.get_parameter('interpolation_distance').value:
                filtered_waypoints.append(new_waypoints[i+1])
                accumulated_distance = 0

        # Check last waypoint distance to the first waypoint
        dist_last_to_first = np.sqrt((filtered_waypoints[-1][0] - filtered_waypoints[0][0]) ** 2 +
                                    (filtered_waypoints[-1][1] - filtered_waypoints[0][1]) ** 2)
        if dist_last_to_first < self.get_parameter('interpolation_distance').value:
            if len(filtered_waypoints) > 1:
                filtered_waypoints.pop()  # Remove last waypoint

        self.interpolated_waypoints = filtered_waypoints

    def plot_interpolated_waypoints(self):
        """Plots the interpolated waypoints"""
        x_interpolated, y_interpolated = zip(*self.interpolated_waypoints)

        plt.figure(figsize=(8, 6))
        plt.plot(*zip(*self.waypoints), 'r-', label='Waypoints')
        plt.plot(x_interpolated, y_interpolated, 'bo', label='Interpolated Points')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Interpolated Waypoints')
        plt.legend()
        plt.grid(True)
        plt.show()

    def save_interpolated_waypoints(self):
        """Saves the interpolated waypoints to a csv file"""
        with open('new_waypoints.csv', 'w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            writer.writerows(self.interpolated_waypoints)

    def calculate_angle(self, point1, point2):
        """Calculates the angle between two points"""
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return np.arctan2(delta_y, delta_x)

    def print_angles_between_waypoints(self):
        """Calculates and prints the angles between consecutive waypoints relative to the angle of the two points before"""
        if len(self.interpolated_waypoints) < 3:
            self.get_logger().info('Not enough waypoints to calculate angles.')
            return

        self.get_logger().info('Angles between consecutive waypoints (relative to angle of two points before):')

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

            self.get_logger().info(f'Waypoint {i} to {i + 1}: {np.degrees(relative_angle):.2f} degrees')

            # Update the previous angle for the next iteration
            previous_angle = angle


def main(args=None):
    rclpy.init(args=args)
    CurveFitting()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
