#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

global waypoint_index
waypoint_index = 5

class WayPointPublisherClient(Node):

    def __init__(self):
        super().__init__('waypoint_publisher_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self,waypoint):
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose = waypoint

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance_remaining))
        if feedback.distance_remaining < 2.0:
            goal_msg = NavigateToPose.Goal()
            
            goal_msg.pose = next_waypoint()

            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

            self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    waypoints = create_waypoints()

    action_client = WayPointPublisherClient()
    for waypoint in waypoints:
        action_client.send_goal(next_waypoint())

        rclpy.spin(action_client)

def next_waypoint():
    global waypoint_index
    data = [
        [11.5,10.5,0.0,math.pi/2,0,0],
        [2.5,15.7,0.0,math.pi,0,0],
        [2.5,11.7,0.0,0,0,0],
        [-3.0,5.0,0.0,math.pi*3/4,0,0],
        [-15.7,15.7,0.0,math.pi,0,0],
        [-11.0,0.7,0.0,0,0,0],
        [0.0,-0.35,0.0,0,0,0]
    ]
    waypoints = []
    for info in data:
        pose = PoseStamped()
        pose.pose._position.x = info[0]
        pose.pose._position.y = info[1]
        pose.pose._position.z = info[2]
        quaternion = quaternion_from_euler(info[3],info[4],info[5])
        pose.pose._orientation.x = quaternion[0]
        pose.pose._orientation.y = quaternion[1]
        pose.pose._orientation.z = quaternion[2]
        pose.pose._orientation.w = quaternion[3]
        waypoints.append(pose)
    waypoint = waypoints[waypoint_index]
    waypoint_index = (waypoint_index+1)%len(waypoints)
    return waypoint

def create_waypoints():
    data = [
        [11.5,10.5,0.0,math.pi/2,0,0],
        # [2.5,15.7,0.0,math.pi,0,0],
        # [2.5,11.7,0.0,0,0,0],
        # [-3.0,5.0,0.0,math.pi*3/4,0,0],
        # [15.7,15.7,0.0,math.pi,0,0],
        # [-11.0,0.7,0.0,0,0,0],
        # [0.0,-0.35,0.0,0,0,0]
    ]
    waypoints = []
    for info in data:
        pose = PoseStamped()
        pose.pose._position.x = info[0]
        pose.pose._position.y = info[1]
        pose.pose._position.z = info[2]
        quaternion = quaternion_from_euler(info[3],info[4],info[5])
        pose.pose._orientation.x = quaternion[0]
        pose.pose._orientation.y = quaternion[1]
        pose.pose._orientation.z = quaternion[2]
        pose.pose._orientation.w = quaternion[3]
        waypoints.append(pose)
    return waypoints

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

if __name__ == '__main__':
    main()