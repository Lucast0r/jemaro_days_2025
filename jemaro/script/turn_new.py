#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math



class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.carposition_x = None
        self.carposition_y = None

        # Create publisher and subscriber
        self.publisher = self.create_publisher(Path, '/path', 10)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.odom_sub = self.create_subscription(Odometry, '/prius/ground_truth', self.odom_callback, qos_profile)
        self.path = Path()
        self.path.header.frame_id = "map"

    def odom_callback(self, msg: Odometry):
        """ Callback to update the car's position """
        self.carposition_x = msg.pose.pose.position.x
        self.carposition_y = msg.pose.pose.position.y
        

    def publish_path(self):
        if self.carposition_x is None or self.carposition_y is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # Parameters for path planning
        avoiding_thresholdy = 15.0
        minavoiding_thresholdx = 2.0
        maxavoiding_thresholdx = 3.0
        obstacle_x = 2.0
        obstacle_y = 0.0
        bufferx = 0.0

        # Prepare the path message
        self.path.poses.clear()
        self.path.header.stamp = self.get_clock().now().to_msg()

        num_points = 5000
        step_y = 0.01 # Step forward in Y

        for i in range(num_points):
            future_y = self.carposition_y + i * step_y
            future_x = self.carposition_x + bufferx 
            distance_y = future_y - obstacle_y
            distance_x = obstacle_x - future_x     

            # Smooth transition logic using sigmoid function
            sigmoid_threshold = (minavoiding_thresholdx + maxavoiding_thresholdx) / 2
            transition_smoothness = 10  # Adjust smoothness of the transition

            # Using sigmoid for smooth turning transition
            if distance_y <= -avoiding_thresholdy:
                dx = 0.0  # Going straight
            elif distance_x < minavoiding_thresholdx and 0 > distance_y >= -avoiding_thresholdy:
                # Smooth left turn
                dx = -0.5
            elif minavoiding_thresholdx <= distance_x <= maxavoiding_thresholdx and -avoiding_thresholdy <= distance_y <= avoiding_thresholdy:
                dx = 0.0  # Go straight
            elif 1.0 < distance_x < maxavoiding_thresholdx and distance_y > avoiding_thresholdy / 2:
                # Smooth right turn
                dx = 0.5
            elif 0 < distance_x < 1 and distance_y > avoiding_thresholdy:
                dx = 0.0  # Go straight after obstacle
            else:
                dx = 0.0  # Default straight path
			
            # Update lateral buffer (car's side-to-side movement)
            bufferx += dx
            #self.get_logger().info(f' Buffer {bufferx} ')
            # Generate pose for each point in the path
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            # Set position for the pose
            pose.pose.position.x = future_x + dx  # The lateral movement
            pose.pose.position.y = future_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            #self.get_logger().info(f' Path X : {pose.pose.position.x} Y : {pose.pose.position.y}  ')
            #self.get_logger().info(f' Distance X  {distance_x} ')
            self.path.poses.append(pose)
				
        # Publish the path
        
        self.publisher.publish(self.path)
        self.get_logger().info(f'Published new path with {len(self.path.poses)} points')


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
