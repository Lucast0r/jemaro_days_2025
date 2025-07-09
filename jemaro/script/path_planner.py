#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import dubins


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.carposition_x = None
        self.carposition_y = None
        self.yaw = 0.0  # vehicle heading

        self.publisher = self.create_publisher(Path, '/path', 10)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.odom_sub = self.create_subscription(Odometry, '/prius/ground_truth', self.odom_callback, qos_profile)
        self.path = Path()
        self.path.header.frame_id = "map"

    def odom_callback(self, msg: Odometry):
        self.carposition_x = msg.pose.pose.position.x
        self.carposition_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        #self.get_logger().info(f"Yaw : {self.yaw}")

    def publish_path(self):
        if self.carposition_x is None or self.carposition_y is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        avoiding_thresholdy = 15.0
        minavoiding_thresholdx = 2.0
        maxavoiding_thresholdx = 3.0
        obstacle_x = 2.0
        obstacle_y = 0.0
        
        self.lastpointposition_x = self.carposition_x
        self.lastpointposition_y = self.carposition_y
        self.path.poses.clear()
        self.path.header.stamp = self.get_clock().now().to_msg()

        num_points = 1000
        step_y = 0.1
		
        for i in range(num_points):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            distance_y = self.lastpointposition_y - obstacle_y
            distance_x = obstacle_x - self.lastpointposition_x
            # self.get_logger().info(f"Distance : {distance_y}")
            # Dubins avoidance condition: trigger when approaching obstacle from behind and near obstacle side
            
            if distance_y <= -avoiding_thresholdy:
                #Go straight before the obstacle
                self.get_logger().info("Going straight")
            
            elif distance_x < minavoiding_thresholdx and 0 > distance_y >= -avoiding_thresholdy:
                #Turn left to avoid the obstacle
                self.get_logger().info("curve initated")
                turning_radius = 25.0
               
                x0 = self.lastpointposition_x
                y0 = self.lastpointposition_y
                
                # Define the two Dubins poses with specified yaws:
                start_pose = (x0, y0, math.pi)  # facing left (-90 deg)
                inter_pose = (x0 - 1.0, y0 + 5 , -3*math.pi / 4)  # 45 deg left
                end_pose = (x0 - 2.0, y0 + 10 , math.pi)  # facing forward, shifted -2m left

                path1 = dubins.shortest_path(start_pose, inter_pose, turning_radius)
                path2 = dubins.shortest_path(inter_pose, end_pose, turning_radius)

                sample_distance = 0.1
                configurations1, _ = path1.sample_many(sample_distance)
                configurations2, _ = path2.sample_many(sample_distance)

        
                for (px, py, pyaw) in configurations1: 
                    #+ configurations2:
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.position.z = 0.0

                    # Convert yaw to quaternion
                    qz = math.sin(pyaw / 2.0)
                    qw = math.cos(pyaw / 2.0)
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw
                    #self.get_logger().info(f"Pose X : {px} Pose Y : {py}")
                    self.path.poses.append(pose)  
                    self.lastpointposition_x = px
                    self.lastpointposition_y = py                  
                
      
            
            elif minavoiding_thresholdx <= distance_x <= maxavoiding_thresholdx and -avoiding_thresholdy <= distance_y <= avoiding_thresholdy:
                #go straight while obstacle is on the side
                self.get_logger().info("Going straight")

            elif 1.0 < distance_x < maxavoiding_thresholdx and distance_y > avoiding_thresholdy / 2:
                #turn right
                self.get_logger().info("curve initated")
                turning_radius = 15.0
               
                x0 = self.lastpointposition_x
                y0 = self.lastpointposition_y
                
                # Define the two Dubins poses with specified yaws:
                start_pose = (x0, y0, math.pi)  # facing left (-90 deg)
                inter_pose = (x0 + 1.0, y0 + 5 , 3*math.pi / 4)  # 45 deg left
                end_pose = (x0 + 2.0, y0 + 10 , math.pi)  # facing forward, shifted -2m left

                path1 = dubins.shortest_path(start_pose, inter_pose, turning_radius)
                path2 = dubins.shortest_path(inter_pose, end_pose, turning_radius)

                sample_distance = 0.05
                configurations1, _ = path1.sample_many(sample_distance)
                configurations2, _ = path2.sample_many(sample_distance)

        
                for (px, py, pyaw) in configurations1: 
                    #+ configurations2:
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.position.z = 0.0

                    # Convert yaw to quaternion
                    qz = math.sin(pyaw / 2.0)
                    qw = math.cos(pyaw / 2.0)
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw
                    #self.get_logger().info(f"Pose X : {px} Pose Y : {py}")
                    self.path.poses.append(pose)  
                    self.lastpointposition_x = px
                    self.lastpointposition_y = py 
            elif 0 < distance_x < 1 and distance_y > avoiding_thresholdy:
                self.get_logger().info("Going straight")
                #Go straight after the obstacle

           
            
            self.lastpointposition_y += step_y
            pose.pose.position.x = self.lastpointposition_x
            pose.pose.position.y = self.lastpointposition_y
            #self.get_logger().info(f"Pose X1 : {pose.pose.position.x} Pose Y1 : {pose.pose.position.y}")
            self.path.poses.append(pose)
            
        self.publisher.publish(self.path)
        self.get_logger().info(f'Published standard path with {len(self.path.poses)} points')


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
