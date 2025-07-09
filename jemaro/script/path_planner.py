#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import dubins
import tf2_ros
import tf2_geometry_msgs

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        # Initializing car's frame origin at (0, 0) with respect to the map
        self.carposition_x = 0.0
        self.carposition_y = 0.0
        self.yaw = 0.0  # Initial car heading (yaw) = 0

        # Publisher setup
        self.publisher = self.create_publisher(Path, '/ZOE3/path_follower/setPath', 10)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.odom_sub = self.create_subscription(Odometry, '/ZOE3/position/map_ekf_odometry', self.odom_callback, qos_profile)
        
        # Path object initialization
        self.path = Path()
        self.path.header.frame_id = "map"

        # Setup tf2 listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg: Odometry):
        """ Callback to update car's position and yaw from the odometry (map frame) """
        self.carposition_x = msg.pose.pose.position.x
        self.carposition_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_path(self):
        """ Function to generate and publish the path in the map frame """
        if self.carposition_x is None or self.carposition_y is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # Define obstacle and avoidance thresholds
        avoiding_thresholdy = 15.0
        minavoiding_thresholdx = 2.0
        maxavoiding_thresholdx = 3.0
        obstacle_x = 2.0
        obstacle_y = 0.0

        # Transform car's current position from map frame to ZOE3/base_link frame
        car_position_in_base_link = self.transform_to_car_frame(self.carposition_x, self.carposition_y)

        # Initialize the path with the car's current position in car frame
        self.lastpointposition_x = car_position_in_base_link[0]
        self.lastpointposition_y = car_position_in_base_link[1]
        self.path.poses.clear()
        self.path.header.stamp = self.get_clock().now().to_msg()

        num_points = 1000
        step_y = 0.1

        for i in range(num_points):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "ZOE3/base_link"  # car frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            distance_y = self.lastpointposition_y - obstacle_y
            distance_x = obstacle_x - self.lastpointposition_x

            # Check avoidance conditions and set path generation logic
            if distance_y <= -avoiding_thresholdy:
                self.get_logger().info("Going straight")
            
            elif distance_x < minavoiding_thresholdx and 0 > distance_y >= -avoiding_thresholdy:
                self.get_logger().info("Curve initiated")
                turning_radius = 25.0
               
                x0 = self.lastpointposition_x
                y0 = self.lastpointposition_y
                
                start_pose = (x0, y0, math.pi)  # facing left (-90 deg)
                inter_pose = (x0 - 1.0, y0 + 5, -3*math.pi / 4)  # 45 deg left
                end_pose = (x0 - 2.0, y0 + 10, math.pi)  # facing forward

                path1 = dubins.shortest_path(start_pose, inter_pose, turning_radius)
                path2 = dubins.shortest_path(inter_pose, end_pose, turning_radius)

                sample_distance = 0.1
                configurations1, _ = path1.sample_many(sample_distance)
                configurations2, _ = path2.sample_many(sample_distance)

                for (px, py, pyaw) in configurations1:
                    # Create a PoseStamped object for the path in the car's local frame (ZOE3/base_link)
                    car_pose = PoseStamped()
                    car_pose.header.frame_id = "ZOE3/base_link"
                    car_pose.pose.position.x = px
                    car_pose.pose.position.y = py
                    car_pose.pose.orientation = self.yaw_to_quaternion(pyaw)

                    # Transform the pose to the map frame
                    transformed_pose = self.transform_pose_to_map_frame(car_pose)

                    # Append the transformed pose to the path
                    self.path.poses.append(transformed_pose)
                    self.lastpointposition_x = px
                    self.lastpointposition_y = py

            # Implement more path generation logic as needed
            # For simplicity, we're reusing similar logic for other cases
            
            self.lastpointposition_y += step_y
            car_pose = PoseStamped()
            car_pose.header.frame_id = "ZOE3/base_link"
            car_pose.pose.position.x = self.lastpointposition_x
            car_pose.pose.position.y = self.lastpointposition_y
            car_pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

            # Transform the pose to the map frame
            transformed_pose = self.transform_pose_to_map_frame(car_pose)
            self.path.poses.append(transformed_pose)

        self.publisher.publish(self.path)
        self.get_logger().info(f'Published standard path with {len(self.path.poses)} points')

    def transform_pose_to_map_frame(self, pose_in_car_frame):
        """ Transform pose from car frame (ZOE3/base_link) to map frame using tf2. """
        try:
            # Get the transform from ZOE3/base_link to map frame
            transform = self.tf_buffer.lookup_transform("map", "ZOE3/base_link", rclpy.time.Time())

            # Transform the pose using tf2
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_in_car_frame, transform)
            return transformed_pose

        except (tf2_ros.TransformException) as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return pose_in_car_frame  # Return the original pose if transformation fails

    def transform_to_car_frame(self, map_x, map_y):
        """ Transform the car's current position from map frame to ZOE3/base_link frame. """
        try:
            # Get the transform from map frame to ZOE3/base_link frame
            transform = self.tf_buffer.lookup_transform("ZOE3/base_link", "map", rclpy.time.Time())

            # Create a PoseStamped message for the position in the map frame
            pose_in_map_frame = PoseStamped()
            pose_in_map_frame.header.frame_id = "map"
            pose_in_map_frame.pose.position.x = map_x
            pose_in_map_frame.pose.position.y = map_y
            pose_in_map_frame.pose.position.z = 0.0
            pose_in_map_frame.pose.orientation = Quaternion()

            # Transform the pose to the car frame (ZOE3/base_link)
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_in_map_frame, transform)

            # Return the transformed position in car frame
            return transformed_pose.pose.position.x, transformed_pose.pose.position.y

        except (tf2_ros.TransformException) as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return map_x, map_y  # Return original position if transform fails
    
    def yaw_to_quaternion(self, yaw):
        """ Convert yaw (radians) to quaternion orientation. """
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
