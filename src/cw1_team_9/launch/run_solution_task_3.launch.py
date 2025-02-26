#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool  
import numpy as np
import math



class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')
        self.get_logger().info("GoToPoiny Node started.")
        # Publisher for waypoint (drives the robot)
        self.waypoint_pub = self.create_publisher(Pose2D, '/gotopoint_waypoint', 10)

        self.obstacle_flag_pub = self.create_publisher(Bool, '/obstacle_flag', 10)

        # Subscribers
        self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.create_subscription(Marker, 'local_map_lines', self.line_callback, 10)

        # Desired goal position
        self.goal_x = -1.5
        self.goal_y = -4.0
        self.goal_theta = 0.0

        # Safety margin
        self.safety_margin = 1.2

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

        # Store obstacle edges
        self.current_edges = []

        # Track obstacle state to avoid redundant publishing
        self.last_obstacle_state = None

        # Timer for control loop
        self.create_timer(1.0, self.control_loop)

    def odom_callback(self, msg):
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def line_callback(self, msg):
        """Process incoming Marker messages from 'local_map_lines' to extract edges."""
        if len(msg.points) < 2:
            self.current_edges = []
            return

        # Convert each point to a NumPy array
        points = [np.array([point.x, point.y]) for point in msg.points]
        # Pair consecutive points to form edges
        self.current_edges = [(points[i], points[i+1]) for i in range(len(points) - 1)]

    def compute_min_distance(self):
        """
        Compute the minimum Euclidean distance from the robot (at (0,0) in its local frame)
        to any edge derived from the Marker messages.
        """
        if not self.current_edges:
            return float('inf')

        robot_pos = np.array([0.0, 0.0])  # Robot's local origin
        min_distance = float('inf')

        for start, end in self.current_edges:
            edge_vector = end - start
            edge_length = np.linalg.norm(edge_vector)
            if edge_length < 0.01:
                continue
            edge_direction = edge_vector / edge_length
            to_robot = robot_pos - start
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            point_on_edge = start + projection * edge_direction
            distance = np.linalg.norm(point_on_edge - robot_pos)
            if distance < min_distance:
                min_distance = distance

        return min_distance

    def control_loop(self):
        """Main control loop."""

        if not self.is_odom_received:
            return

        min_distance = self.compute_min_distance()
        self.get_logger().info(f"Minimum distance to obstacle: {min_distance:.2f} m")

        # Prepare the obstacle flag message
        obstacle_detected = min_distance < self.safety_margin
        obstacle_flag_msg = Bool()
        obstacle_flag_msg.data = bool(obstacle_detected)

        if obstacle_detected != self.last_obstacle_state:
            self.obstacle_flag_pub.publish(obstacle_flag_msg)
            state = "detected" if obstacle_detected else "cleared"
            self.get_logger().info(f"Obstacle {state}. Published flag: {obstacle_detected}")
            self.last_obstacle_state = obstacle_detected

        if obstacle_detected:
            self.get_logger().info("Obstacle detected! Stopping robot.")
            stop_msg = Pose2D(x=self.current_x, y=self.current_y, theta=self.current_orientation)
            self.waypoint_pub.publish(stop_msg)
        else:
            self.get_logger().info("No obstacle detected. Moving toward goal.")
            goal_msg = Pose2D(x=self.goal_x, y=self.goal_y, theta=self.goal_theta)
            self.waypoint_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
