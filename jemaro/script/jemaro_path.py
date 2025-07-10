#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')
        
        # Publisher for path (nav_msgs/Path is a series of PoseStamped messages)
        self.publisher = self.create_publisher(Path, '/path', 100)
        
        # Timer to trigger path publishing periodically
        self.timer = self.create_timer(1.0, self.publish_path)
        
        # Initialize path message
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = self.get_clock().now().to_msg()

    def publish_path(self):
        # Clear existing poses (for each new publish cycle)
        self.path.poses.clear()
        
        # Create a series of poses in a loop
        for i in range(100):  # Example: 10 poses along the X-axis
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            
            # Define positions (simple straight line for this example)
            pose.pose.position.x = i * -0.1  # X moves from 0 to 4.5 meters
            pose.pose.position.y = -i * 0.5
            pose.pose.position.z = 0.0

            # Define orientation (robot facing forward, no rotation)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            # Append to path
            self.path.poses.append(pose)
        
        # Publish the complete path
        self.path.header.stamp = self.get_clock().now().to_msg()  # Update header timestamp
        self.publisher.publish(self.path)
        self.get_logger().info('Published new path')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the PathPublisher node
    node = PathPublisher()

    # Keep the node running
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
