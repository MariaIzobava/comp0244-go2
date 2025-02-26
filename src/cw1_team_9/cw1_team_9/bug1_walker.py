#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math

class Bug1Algorithm(Node):
    def __init__(self):
        self.received_first_edge_wp = False
        # Flag to indicate that the hit point is reached and mission is complete.
        self.mission_complete = False
        super().__init__('bug1_algorithm')
        self.get_logger().info("Bug1 Algorithm Node started.")

        # Variables to store the latest waypoints.
        self.gotopoint_wp = None
        self.edge_wp = None

        # Obstacle flag: False = go-to-point, True = edge follower.
        self.obstacle_flag = False

        # Variables used for Bug1 algorithm.
        self.starting_point = None    # Record the first edge waypoint when edge following starts.
        self.edge_wp_counter = 0
        self.circling_finished = False

        # The goal position is published by the gotopoint_waypoint.
        self.goal_position = (-1.5, -4.0)  

        self.min_distance_to_goal = float('inf')  
        self.hit_point = None                    
        self.hit_point_reached = False            

        # Mode lock: once switched to EDGE_FOLLOWER, remain there until conditions are met.
        self.edge_follower_locked = False

        # Subscriptions.
        self.create_subscription(Pose2D, '/gotopoint_waypoint', self.gotopoint_callback, 10)
        # Change suppressed_waypoint to /edge_waypoint if using default clockwise edge_follower
        self.create_subscription(Pose2D, "suppressed_waypoint", self.edge_callback, 10)
        self.create_subscription(Bool, '/obstacle_flag', self.flag_callback, 10)

        # Publisher.
        self.waypoint_pub = self.create_publisher(Pose2D, '/waypoint', 10)
        
        self.waypoint_pub = self.create_publisher(Bool, 'suppress_edge_follower', 10)
        msg = Bool()
        msg.data = True  
        self.waypoint_pub.publish(msg)


    def calculate_distance(self, p1, p2):
        """Compute Euclidean distance between two points (each a tuple (x,y))."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def gotopoint_callback(self, msg: Pose2D):
        # Update the go-to-point waypoint and goal position.
        if self.mission_complete:
            return  
        self.gotopoint_wp = msg
        self.goal_position = (msg.x, msg.y)
        self.forward_waypoint()

    def edge_callback(self, msg: Pose2D):
        if self.mission_complete:
            return  
        # Store the latest edge waypoint.
        self.edge_wp = msg
        self.received_first_edge_wp = True

        if self.edge_follower_locked:
            self.edge_wp_counter += 1

            if not self.circling_finished:
                # Record the starting point once enough edge waypoints have been received.
                if self.starting_point is None and self.edge_wp_counter >= 20:
                    self.starting_point = msg
                    self.get_logger().info(f"Starting point recorded at: x={msg.x:.2f}, y={msg.y:.2f}")

                # Calculate distance from the current edge waypoint to the goal.
                current_distance = self.calculate_distance((msg.x, msg.y), self.goal_position)

                # Update the hit point if this waypoint is closer to the goal.
                if current_distance < self.min_distance_to_goal:
                    self.min_distance_to_goal = current_distance
                    self.hit_point = msg
                    self.get_logger().info(
                        f"New best hit point: x={msg.x:.2f}, y={msg.y:.2f}, distance: {current_distance:.2f}"
                    )

                # Check if the robot has circumnavigated the obstacle.
                if self.starting_point is not None and self.edge_wp_counter >= 50:
                    distance_from_start = self.calculate_distance(
                        (msg.x, msg.y), (self.starting_point.x, self.starting_point.y)
                    )
                    if distance_from_start < 0.4:  # threshold in meters
                        self.get_logger().info("Circling finished. Fixing hit point.")
                        self.circling_finished = True
                        self.get_logger().info(
                            f"Fixed hit point at: x={self.hit_point.x:.2f}, y={self.hit_point.y:.2f}"
                        )
            else:
                # After circling, compare the current edge waypoint with the hit point.
                if self.hit_point is not None:
                    diff = self.calculate_distance((msg.x, msg.y), (self.hit_point.x, self.hit_point.y))
                    self.get_logger().info(f"Distance to fixed hit point: {diff:.2f}")
                    if diff < 0.3:
                        if not self.hit_point_reached:
                            self.hit_point_reached = True
                            self.get_logger().info("Hit point reached; stopping robot.")
                            # Publish current position to stop the robot.
                            self.waypoint_pub.publish(msg)
                        return  # Exit early.
        self.forward_waypoint()

    def flag_callback(self, msg: Bool):
        if self.mission_complete:
            return
        
        if self.hit_point_reached:
            self.get_logger().info("Hit point reached; unlocking EDGE_FOLLOWER mode and switching to GO-TO-POINT.")
            self.edge_follower_locked = False  
            if self.gotopoint_wp is not None:
                self.waypoint_pub.publish(self.gotopoint_wp)
                self.get_logger().info("Published GO-TO-POINT waypoint from flag callback.")
            return  

        # Control mode based on obstacle detection.
        if msg.data:
            if not self.edge_follower_locked:
                self.edge_follower_locked = True
                self.get_logger().info("Obstacle detected! Switching to EDGE_FOLLOWER mode (locked).")
            else:
                self.get_logger().info("Obstacle still detected. Remaining in EDGE_FOLLOWER mode.")
        else:
            if self.edge_follower_locked:
                self.get_logger().info("No obstacle detected! Switching to GO-TO-POINT mode immediately.")
                self.edge_follower_locked = False
                if self.gotopoint_wp is not None:
                    self.waypoint_pub.publish(self.gotopoint_wp)
                    self.get_logger().info("Published GO-TO-POINT waypoint from flag callback.")
            self.obstacle_flag = False

        self.forward_waypoint()


    def forward_waypoint(self):
        # If mission is complete, only publish the goal.
        if self.mission_complete:
            if self.gotopoint_wp is not None:
                self.waypoint_pub.publish(self.gotopoint_wp)
                self.get_logger().info("Mission complete: Publishing final GO-TO-POINT waypoint.")
            return

        # If circling is finished and hit point is reached, publish the hit point (i.e. stop).
        if self.circling_finished and self.hit_point_reached:
            self.waypoint_pub.publish(self.hit_point)
            self.get_logger().info("Hit point reached; robot stopped.")
            return

        # Otherwise, forward the appropriate waypoint.
        if self.edge_follower_locked:
            if self.edge_wp is not None and self.received_first_edge_wp:
                self.waypoint_pub.publish(self.edge_wp)
                self.get_logger().info("Forwarded EDGE_FOLLOWER waypoint.")
            else:
                self.get_logger().warn("Locked in EDGE_FOLLOWER mode, but no EDGE_FOLLOWER waypoint available!")
        else:
            if self.obstacle_flag:
                if self.edge_wp is not None:
                    self.waypoint_pub.publish(self.edge_wp)
                    self.get_logger().info("Forwarded EDGE_FOLLOWER waypoint (obstacle active).")
                else:
                    self.get_logger().warn("Obstacle active, but no EDGE_FOLLOWER waypoint available!")
            else:
                if self.gotopoint_wp is not None:
                    self.waypoint_pub.publish(self.gotopoint_wp)
                    self.get_logger().info("Forwarded GO-TO-POINT waypoint.")
                else:
                    self.get_logger().warn("No GO-TO-POINT waypoint available!")

def main(args=None):
    rclpy.init(args=args)
    node = Bug1Algorithm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
