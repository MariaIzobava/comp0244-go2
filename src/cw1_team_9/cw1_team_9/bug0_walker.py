import rclpy
import numpy as np
from rclpy.node import Node
from collections import deque
import math


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

        self.UPDATE_RATE = 0.5  # seconds

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.walking_type = STAY
        self.arrived = True
        self._obstructive_segments = deque()

        # Robot current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.pause_edge_follower_pub = self.create_publisher(Bool, 'pause_edge_follower', 10)
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
        if self.goal_x==None or abs(self.goal_x-msg.x) > 0.2 or abs(self.goal_y-msg.y) > 0.2 or abs(self.goal_theta-msg.theta) > 0.2:
            self.arrived = False

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

        # Adjusting current position to the center of the robot. The lidar 
        # is located on its head.
        self.current_x -= math.cos(self.current_orientation) * 0.2
        self.current_y -= math.sin(self.current_orientation) * 0.2


    def timer_callback(self):
        """Main control loop"""

        # If we haven't received a GOAL yet then do nothing.
        if self.goal_x is None:
            self.get_logger().info('Bug0: Stay put. No GOAL yet.')
            return

        A = Point2D(self.current_x, self.current_y)
        B = Point2D(self.goal_x, self.goal_y)

        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 0
        edge_marker.scale.x = 0.05
        edge_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        edge_marker.points.append(Point(x=A.x, y=A.y, z=0.0))
        edge_marker.points.append(Point(x=B.x, y=B.y, z=0.0))
        self.goal_vector_pub.publish(edge_marker)

        self.get_logger().info(f'PUBLISHING: {A} {B}')

        while len(self._obstructive_segments) > 0:
            segment = self._obstructive_segments.popleft()
            C = segment[0]
            D = segment[1]
            if intersect(A, B, C, D):
                self._obstructive_segments.appendleft(segment)
                break
            else:
                self.get_logger().info(f"NO LONGER INTERSECTS: {A} {B} {C} {D}")

        if len(self._obstructive_segments) == 0:
            if self.walking_type == STRAIGHT:
                 self.get_logger().info("KEEP STRAIGHT: no obstacles")
            else:
                self.get_logger().info("START STRAIGHT: no more obstacles")
                self.go_with_waypoint_follower()
        else:
            if self.walking_type == AROUND:
                self.get_logger().info("KEEP AROUND: there is an obstacle")
            else:
                self.get_logger().info("START AROUND: found an obstacle")
                self.go_with_edge_follower()
                
        

    # This is the main callback which decides what the robot should do:
    # 1. follow straight towards the goal
    # 2. go around the edge of an obstacle
    #
    # We decide by checking whether any of the segments we see in front of us 
    # intersect with our straight route towards the goal.
    def line_callback(self, msg):
        """Process incoming line segments"""

        if self.goal_x is None:
            return

        # Convert line points to numpy arrays
        points = [np.array([point.x, point.y]) for point in msg.points]

        A = Point2D(self.current_x, self.current_y)
        B = Point2D(self.goal_x, self.goal_y)

        start_point = points[0]
        end_point = points[len(points) - 1]
        safety_margin_angle = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        safety_margin_x = 1.0 * math.cos(safety_margin_angle)
        safety_margin_y = 1.0 * math.sin(safety_margin_angle)

        C = Point2D(start_point[0] - safety_margin_x, start_point[1] - safety_margin_y)
        D = Point2D(end_point[0] + safety_margin_x, end_point[1] + safety_margin_y)

        if intersect(A, B, C, D):
            self.get_logger().info("INTERSECTS!")
            self._obstructive_segments.append([C, D])

            # Detected edges
            edge_marker = Marker()
            edge_marker.header.frame_id = "livox"
            edge_marker.header.stamp = self.get_clock().now().to_msg()
            edge_marker.type = Marker.LINE_LIST
            edge_marker.action = Marker.ADD
            edge_marker.id = 0
            edge_marker.scale.x = 0.05
            edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            
            edge_marker.points.append(Point(x=C.x, y=C.y, z=0.0))
            edge_marker.points.append(Point(x=D.x, y=D.y, z=0.0))


            
            self.edge_marker_pub.publish(edge_marker)
        else:
            self.get_logger().info("DON'T INTERSECT!")
            self._obstructive_segments.append([C, D])

            # Detected edges
            edge_marker = Marker()
            edge_marker.header.frame_id = "livox"
            edge_marker.header.stamp = self.get_clock().now().to_msg()
            edge_marker.type = Marker.LINE_LIST
            edge_marker.action = Marker.ADD
            edge_marker.id = 0
            edge_marker.scale.x = 0.05
            edge_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            edge_marker.points.append(Point(x=C.x, y=C.y, z=0.0))
            edge_marker.points.append(Point(x=D.x, y=D.y, z=0.0))
            
            self.edge_marker_pub.publish(edge_marker)


    def go_with_waypoint_follower(self):
        # 1. Stop following the edge
        p = Bool()
        p.data = True
        self.pause_edge_follower_pub.publish(p)

        # 2. Publish the GOAL as the next waypoint
        waypoint_msg = Pose2D()
        waypoint_msg.x = float(self.goal_x)
        waypoint_msg.y = float(self.goal_y)
        waypoint_msg.theta = self.goal_theta
        self.waypoint_pub.publish(waypoint_msg)
        self.walking_type = STRAIGHT

        self.get_logger().info('Bug0: START going STRAIGHT, no obstacles.')


    def go_with_edge_follower(self):
        # Start following the nearest edge
        p = Bool()
        p.data = False
        self.pause_edge_follower_pub.publish(p)
        self.walking_type = AROUND
        self.get_logger().info('Bug0: START going AROUND the obstacle.')


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