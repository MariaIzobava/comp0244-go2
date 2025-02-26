import math
import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from dataclasses import dataclass
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


@dataclass
class Point2D:
    x: float
    y: float


def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)


# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


STAY = 'stay'
STRAIGHT = 'straight'
AROUND = 'around'


class Bug0Walker(Node):

    def __init__(self):
        super().__init__('bug0_node')
        self.get_logger().info('Bug0: I am ready')

        self.SAFETY_MARGIN = 1.0  # meters
        self.INCREMENT_DISTANCE = 0.7 # meters
        self.UPDATE_RATE = 0.5  # seconds

        # This value is used both for waypoint overshoot (needed when we
        # walk along the cube-like objects) and for "goal vector" start
        # point adjustment. Value for overshoot must not be smaller than
        # value for adjustment. It's obvious when looking at the visualised
        # goal vector and detected edge. 0.4 was picked empirically 
        # and should satisfy both cases.  
        self.WAYPOINT_OVERSHOOT = 0.4  # meters 

        # This value is used to "extend" detected edge when checking
        # for its intersection with the goal vector. Without it
        # the robot stops going around the obstacle too soon
        # and can touch the obstacle. 0.2 is also pretty small value, 
        # looks like it's just half of the width of the robot so it still
        # walks rather close to the obstacle. However, during the testing
        # it was enough for the robot not to touch the obstacle. Try increasing
        # it if the robot bumps into something:) 
        self.SAFETY_SEGMENT = 0.2  # meters

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        # Helper fields
        self.walking_type = STAY
        self.has_obstructive_segment = False
        self.current_edges = None
        self.current_edge_safety_margins = None

        # Robot current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)
        self.goal_vector_pub = self.create_publisher(Marker, 'goal_vector', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.odom_callback,
            10)
        self.line_sub = self.create_subscription(
            Marker,
            'local_map_lines',
            self.line_callback,
            10)

        self.goal_sub = self.create_subscription(
            Pose2D,
            '/goal',
            self.goal_callback,
            10
        )

        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)


    def goal_callback(self, msg: Pose2D):
        """
        Updates the target waypoint when a new message is received.
        """
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta
        # STOP the robot so it can start going to the new GOAL
        self.walking_type = STAY
        self.get_logger().info(
            f"Received new GOAL: x={self.goal_x}, y={self.goal_y}, orientation={self.goal_theta}. current state: x={self.current_x}, y={self.current_y}, orientation={self.current_orientation}"
        )

    def odom_callback(self, msg):
        """Update current robot pose (x,y,theta) from odometry (in odom frame)"""
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)


    def timer_callback(self):
        """Main control loop"""

        # If we haven't received a GOAL yet then do nothing.
        if self.goal_x is None:
            self.get_logger().info('Bug0: Stay put. No GOAL yet.')
            return

        A, B = self.get_goal_vector()
        goal_vector_marker = self.get_segment_marker(A, B, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        self.goal_vector_pub.publish(goal_vector_marker)

        if self.has_obstructive_segment:
            intersects = False
            for edge in self.current_edges:
                C = Point2D(edge[0][0], edge[0][1])
                D = Point2D(edge[1][0], edge[1][1])
                if intersect(A, B, C, D):
                    intersects = True
                    break

            if not intersects:
                # Also checking safety margins so the robot doesn't walk too close
                # to the obstacle when finally sees the GOAL:
                intersects = intersect(A, B, self.current_edge_safety_margins[0][0], self.current_edge_safety_margins[0][1]) or intersects
                intersects = intersect(A, B, self.current_edge_safety_margins[1][0], self.current_edge_safety_margins[1][1]) or intersects

            if not intersects:
                self.has_obstructive_segment = False

        if not self.has_obstructive_segment:
            if self.walking_type != STRAIGHT:
                 self.get_logger().info("START STRAIGHT: no more obstacles")
            self.go_straight()
        else:
            if self.walking_type != AROUND:
                self.get_logger().info("START AROUND: found an obstacle")
            self.go_around()


    def get_goal_vector(self):
        # Adjusting current position to the BACK of the robot. The lidar 
        # is located on its head so the odometry is related to it.
        robot_center_x = self.current_x - math.cos(self.current_orientation) * self.WAYPOINT_OVERSHOOT
        robot_center_y = self.current_y - math.sin(self.current_orientation) * self.WAYPOINT_OVERSHOOT
        local_a = self.transform_to_base_link([robot_center_x, robot_center_y])
        local_b = self.transform_to_base_link([self.goal_x, self.goal_y])
        A = Point2D(local_a[0], local_a[1])
        B = Point2D(local_b[0], local_b[1])
        return (A, B)


    def get_empty_segment_marker(self, color: ColorRGBA):
        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 0
        edge_marker.scale.x = 0.05
        edge_marker.color = color
        return edge_marker

    def get_segment_marker(self, a: Point2D, b: Point2D, color: ColorRGBA):
        edge_marker = self.get_empty_segment_marker(color)
        edge_marker.points.append(Point(x=float(a.x), y=float(a.y), z=0.0))
        edge_marker.points.append(Point(x=float(b.x), y=float(b.y), z=0.0))
        return edge_marker

                
    def transform_to_camera_init(self, point):
        """Transform a point from livox to camera_init frame, which is our fixed world frame"""
        # Rotation matrix
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        
        # Transform point
        x = point[0] * c - point[1] * s + self.current_x
        y = point[0] * s + point[1] * c + self.current_y
        
        return np.array([x, y])


    def transform_to_base_link(self, point):
        """Transform a point from camera_init to livox frame"""
        # Translate to origin
        dx = point[0] - self.current_x
        dy = point[1] - self.current_y
        
        # Rotation matrix inverse
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        
        # Transform point
        x = dx * c - dy * s
        y = dx * s + dy * c
        
        return np.array([x, y])
        

    def line_callback(self, msg):
        """Process incoming line segments"""

        if self.goal_x is None:
            return

        if len(msg.points) < 2:
            return

        A, B = self.get_goal_vector()

        # Convert line points to numpy arrays
        points = [np.array([point.x, point.y]) for point in msg.points]

        # Take the first and the last points of the edge.
        start_point = points[0]
        end_point = points[len(points) - 1]

        # Extending the segment from both sides for safety.
        safety_margin_angle = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        safety_margin_x = self.SAFETY_SEGMENT * math.cos(safety_margin_angle)
        safety_margin_y = self.SAFETY_SEGMENT * math.sin(safety_margin_angle)

        start_point_C = Point2D(start_point[0], start_point[1])
        end_point_C = Point2D(end_point[0], end_point[1])
        start_point_D = Point2D(start_point[0] - safety_margin_x, start_point[1] - safety_margin_y)
        end_point_D = Point2D(end_point[0] + safety_margin_x, end_point[1] + safety_margin_y)

        safety_edges = []
        safety_edges.append([start_point_C, start_point_D])
        safety_edges.append([end_point_C, end_point_D])

        edges = []
        for i in range(len(points) - 1):
            edges.append((points[i], points[i+1]))
        intersects = False
        for edge in edges:
            C = Point2D(edge[0][0], edge[0][1])
            D = Point2D(edge[1][0], edge[1][1])
            if intersect(A, B, C, D):
                intersects = True
                break

        intersects = intersect(A, B, safety_edges[0][0], safety_edges[0][1]) or intersects
        intersects = intersect(A, B, safety_edges[1][0], safety_edges[1][1]) or intersects

        if intersects:
            # If the segment intersects with our goal vector we use
            # it as a new obstructive segment around which we need to go.
            self.has_obstructive_segment = True
            self.current_edges = edges
            self.current_edge_safety_margins = safety_edges
                
            edge_marker_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        else:
            edge_marker_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        edge_marker = self.get_empty_segment_marker(edge_marker_color)
        for i in range(len(points) - 1):
            edge_marker.points.append(Point(x=points[i][0], y=points[i][1], z=0.0))
            edge_marker.points.append(Point(x=points[i+1][0], y=points[i+1][1], z=0.0))
        edge_marker.points.append(Point(x=start_point_D.x, y=start_point_D.y, z=0.0))
        edge_marker.points.append(Point(x=end_point_D.x, y=end_point_D.y, z=0.0))
        self.edge_marker_pub.publish(edge_marker)


    # This function is full copy from original edge_follower node
    def find_next_waypoint(self):
        """Find closest edge and calculate next waypoint"""
        if not self.current_edges:
            return None
            
        # Robot is at origin in livox frame
        robot_pos = np.array([0.0, 0.0])
        
        # First find the closest point on any edge segment
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            
            # Calculate edge vector
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            
            if edge_length < 0.01:  # Skip very short edges
                continue
                
            # Normalize edge vector
            edge_direction = edge_vector / edge_length
            
            # Vector from start point to robot
            to_robot = robot_pos - start_point
            
            # Project robot position onto edge line
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            point_on_edge = start_point + projection * edge_direction
            
            # Calculate distance to edge
            distance = np.linalg.norm(point_on_edge - robot_pos)
            
            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point_on_edge
                closest_edge_index = i
        
        if closest_edge is None:
            return None

        # Store the closest point (red dot)
        self.closest_edge_point = closest_point

        # Now move clockwise along the continuous edge by INCREMENT_DISTANCE
        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        
        # Vector from closest point to robot
        to_robot = robot_pos - closest_point
        
        # Determine cw direction using cross product
        cross_z = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        moving_forward = cross_z > 0
        
        # Move along edges to find increment point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point
        
        if moving_forward:
            while increment_left > 0 and current_index < len(self.current_edges):
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(end - current_point)
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point + edge_direction * increment_left
                    break
                else:
                    # Move to next edge
                    increment_left -= remaining_distance
                    current_index += 1
                    if current_index >= len(self.current_edges):
                        # If we reach the end, stay at the last point
                        current_index = len(self.current_edges) - 1
                        current_point = self.current_edges[current_index][1]
                        break
        else:
            while increment_left > 0 and current_index >= 0:
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(current_point - start)
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point - edge_direction * increment_left
                    break
                else:
                    # Move to previous edge
                    increment_left -= remaining_distance
                    current_index -= 1
                    if current_index < 0:
                        # If we reach the start, stay at the first point
                        current_index = 0
                        current_point = self.current_edges[current_index][0]
                        break

        # Store the incremented point (green dot)
        self.incremented_point = current_point
        
        # Get the edge direction at the incremented point
        current_edge = self.current_edges[current_index]
        start, end = current_edge
        edge_direction = (end - start) / np.linalg.norm(end - start)
        
        # Calculate perpendicular vector (rotate edge_direction 90 degrees)
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        
        # Check which side the robot is on
        to_robot = robot_pos - current_point
        if np.dot(perpendicular, to_robot) < 0:
            perpendicular = -perpendicular  # Flip if needed to point toward robot's side
            
        # Calculate waypoint by projecting perpendicular to the edge
        waypoint = current_point + perpendicular * self.SAFETY_MARGIN

        if moving_forward:
            waypoint[0] += self.WAYPOINT_OVERSHOOT * math.cos(math.atan2(edge_direction[1], edge_direction[0]))
            waypoint[0] += self.WAYPOINT_OVERSHOOT * math.sin(math.atan2(edge_direction[1], edge_direction[0]))
        else:
            waypoint[0] -= self.WAYPOINT_OVERSHOOT * math.cos(math.atan2(edge_direction[1], edge_direction[0]))
            waypoint[0] -= self.WAYPOINT_OVERSHOOT * math.sin(math.atan2(edge_direction[1], edge_direction[0]))
        
        return waypoint


    def go_straight(self):
        # Publish the GOAL as the next waypoint
        waypoint_msg = Pose2D()
        waypoint_msg.x = float(self.goal_x)
        waypoint_msg.y = float(self.goal_y)
        waypoint_msg.theta = self.goal_theta
        self.waypoint_pub.publish(waypoint_msg)
        self.walking_type = STRAIGHT


    def go_around(self):
        self.walking_type = AROUND

        next_waypoint = self.find_next_waypoint()
        
        # Publish waypoint if valid
        if next_waypoint is not None:
            waypoint_msg = Pose2D()
            waypoint_camera_init = self.transform_to_camera_init(next_waypoint)
            waypoint_msg.x = float(waypoint_camera_init[0])
            waypoint_msg.y = float(waypoint_camera_init[1])
            waypoint_msg.theta = self.current_orientation  # Keep current orientation
            
            self.waypoint_pub.publish(waypoint_msg)
        else:
            self.get_logger().info("NO waypoint!")


def main(args=None):
    rclpy.init(args=args)

    walker = Bug0Walker()

    rclpy.spin(walker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    walker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()