#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.carposition_x = None
        self.carposition_y = None
      
        
        self.publisher = self.create_publisher(Path, '/path', 10)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.odom_sub = self.create_subscription(Odometry, '/prius/ground_truth', self.odom_callback, qos_profile)
        self.path = Path()
        self.path.header.frame_id = "map"

    def odom_callback(self, msg: Odometry):
        self.carposition_x = msg.pose.pose.position.x
        self.carposition_y = msg.pose.pose.position.y
            
         
    def publish_path(self):
        if self.carposition_x is None or self.carposition_y is None:
            self.get_logger().warn("Waiting for odometry data...")
            return
        avoiding_thresholdy = 10.0
        minavoiding_thresholdx = 1
        maxavoiding_thresholdx = 2.5
        self.path.poses.clear()
        obstacle_x = 2
        obstacle_y = 0
        self.get_logger().info(f"Position X: {self.carposition_x}")
        self.get_logger().info(f"Position Y: {self.carposition_y}")

        distance_y = self.carposition_y - obstacle_y
        distance_x = obstacle_x - self.carposition_x

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Path decision based on distances
        if distance_y <= - avoiding_thresholdy:
            self.get_logger().info("We go straight")
            pose.pose.position.x = self.carposition_x + 0.0
            pose.pose.position.y = self.carposition_y + 5.0

        elif distance_x < minavoiding_thresholdx and 0 > distance_y >= - avoiding_thresholdy:
            self.get_logger().info("We turn left")
            pose.pose.position.x =  self.carposition_x -1.0
            pose.pose.position.y = self.carposition_y + 5.0

        elif minavoiding_thresholdx <= distance_x <= maxavoiding_thresholdx and -avoiding_thresholdy <= distance_y <= avoiding_thresholdy:
            self.get_logger().info("We go straight again")
            pose.pose.position.x =  self.carposition_x + 0.0
            pose.pose.position.y = self.carposition_y + 5.0

        elif minavoiding_thresholdx< distance_x < maxavoiding_thresholdx and distance_y > avoiding_thresholdy:
            self.get_logger().info("We turn right")
            pose.pose.position.x =  self.carposition_x+ 0.8
            pose.pose.position.y = self.carposition_y+ 5.0

        elif 0  <distance_x < 1  and distance_y > avoiding_thresholdy:
            self.get_logger().info("Go straight after obstacle")
            pose.pose.position.x =  self.carposition_x +0.0
            pose.pose.position.y =  self.carposition_y +5.0
        else:
            self.get_logger().info("No case")
			    
        """
        elif distance_x < minavoiding_thresholdx and -avoiding_thresholdy <= distance_y <= avoiding_thresholdy:
            self.get_logger().info("Overshoot right")
            pose.pose.position.x =  self.carposition_x+ 1.0
            pose.pose.position.y = self.carposition_y+ 5.0
        """

        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path)
        self.get_logger().info('Published new path')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
