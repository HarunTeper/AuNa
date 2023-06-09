#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt


class WaypointPublisher(Node):
    """Waypoint publisher node"""
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.timer = self.create_timer(0.5, self.timer_callback)

        # get the file path of the csv file from a ros parameter
        self.declare_parameter('waypoint_file', '~/auna_ws/src/auna_physical/config/flw_waypoints.csv')
        self.waypoint_file_path = self.get_parameter('waypoint_file').value

        # declare parameter for filter distance
        self.declare_parameter('filter_distance', 1.0)
        self.filter_distance = self.get_parameter('filter_distance').value
        self.declare_parameter('plot_results', False)
        self.plot_results = self.get_parameter('plot_results').value
        self.declare_parameter('swap_xy', False)
        self.swap_xy = self.get_parameter('swap_xy').value

    def timer_callback(self):
        """Timer callback function."""

        #cancel the timer
        self.timer.cancel()

        with open(self.waypoint_file_path, 'r', encoding='utf-8') as file:
            waypoints = file.readlines()
            waypoints = [line.strip().split(',') for line in waypoints]
            if self.swap_xy:
                waypoints = [[float(point[1]), float(point[0])] for point in waypoints]
            else:
                waypoints = [[float(point[0]), float(point[1])] for point in waypoints]

        # save the x and y coordinates separately
        x = [point[0] for point in waypoints]
        y = [point[1] for point in waypoints]

        # get the angle for each point by calculating the angle between the previous and next point
        angle = []
        for i in range(len(waypoints)):
            if i == 0:
                angle.append(np.arctan2(y[i+1] - y[-1], x[i+1] - x[-1]))
            elif i == len(waypoints) - 1:
                angle.append(np.arctan2(y[0] - y[i-1], x[0] - x[i-1]))
            else:
                angle.append(np.arctan2(y[i+1] - y[i-1], x[i+1] - x[i-1]))

        #print dimensions of np.array((x, y))
        print(np.array((x, y)).shape)

        # create a 2D interpolator
        interpolator = interpolate.NearestNDInterpolator(np.array((x, y)).T, np.array(angle))

        # get the interpolated results at the old waypoints
        interpolated_angle = interpolator(np.array(x), np.array(y))

        # starting from waypoint 0 in waypoints, sample new waypoints at 0.1m intervals

        new_waypoints = []
        new_angles = []

        new_waypoint =  waypoints[0]
        new_angle = interpolated_angle[0]

        steps = 1000
        step_size = 0.001

        # until the new waypoint is close to the last waypoint
        while np.sqrt((new_waypoint[0] - waypoints[0][0])**2 + (new_waypoint[1] - waypoints[0][1])**2) > 0.05 or len(new_waypoints) < steps:
            # sample a new waypoint
            new_waypoint = [new_waypoint[0] + step_size*np.cos(new_angle), new_waypoint[1] + step_size*np.sin(new_angle)]

            # get the new angle
            new_angle = interpolator(np.array(new_waypoint[0]), np.array(new_waypoint[1]))

            #save the new waypoint and angle in a new list
            new_waypoints.append(new_waypoint)
            new_angles.append(new_angle)

        # filter the waypoints by accumulating the distance between consecutive waypoints until it is greater than the filter distance
        filtered_waypoints = [new_waypoints[0]]
        i = 0
        distance = 0
        while i < len(new_waypoints) - 1:
            distance += np.sqrt((new_waypoints[i+1][0] - new_waypoints[i][0])**2 + (new_waypoints[i+1][1] - new_waypoints[i][1])**2)
            if distance > self.filter_distance:
                filtered_waypoints.append(new_waypoints[i+1])
                distance = 0
            i += 1

        # remove the last waypoint if it is too close to the first waypoint
        if np.sqrt((filtered_waypoints[-1][0] - filtered_waypoints[0][0])**2 + (filtered_waypoints[-1][1] - filtered_waypoints[0][1])**2) < self.filter_distance:
            filtered_waypoints.pop()

        # save new waypoints to a csv file
        with open('new_waypoints.csv', 'w', encoding='utf-8') as file:
            for point in filtered_waypoints:
                file.write(str(point[0]) + ',' + str(point[1]) + '\n')

        # Print that the new waypoints have been saved using rclpy
        self.get_logger().info('New waypoints saved to new_waypoints.csv')

        # plot the filtered waypoints
        if self.plot_results:
            plt.figure()
            plt.plot(x, y, 'go')
            plt.plot([point[0] for point in filtered_waypoints], [point[1] for point in filtered_waypoints], 'ro')
            plt.show()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
