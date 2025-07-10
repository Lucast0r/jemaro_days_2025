#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
import math 

import dubins
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        # Initializing car's frame origin at (0, 0) with respect to the map
        self.carposition_x = None
        self.carposition_y = None
        self.yaw = 0.0  # Initial car heading (yaw) = 0
        self.cone_x = 0.0
        self.cone_y = 0.0
        self.sum_y = 0.0
        # Publisher setup
        #self.publisher = self.create_publisher(Path, '/ZOE3/path_follower/setPath', 1000)
        self.publisher = self.create_publisher(Path, '/path', 5000)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.odom_sub = self.create_subscription(Odometry, '/ZOE3/position/map_ekf_odometry', self.odom_callback, qos_profile)
        self.odom_sub = self.create_subscription(PoseStamped, '/conePosition', self.cone_callback, qos_profile)

        # Path object initialization
        self.path = Path()
        self.path.header.frame_id = 'map'

        # Setup tf2 listener for transformations with 100s cache
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg: Odometry):
        """ Callback to update car's position and yaw from the odometry (map frame) """
        
        

        self.carposition_x = msg.pose.pose.position.x
        self.carposition_y = msg.pose.pose.position.y
		
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q)
        
    def cone_callback(self, msg: Odometry):
        cone_pose_stamped = PoseStamped()
        cone_pose_stamped.header.frame_id = 'map'
        cone_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        cone_pose_stamped.pose = msg.pose 
        cone_pose_stamped.pose.position.x 
        
        transform = self.tf_buffer.lookup_transform('ZOE3/base_link', 'map', rclpy.time.Time())
        # Transform full PoseStamped
        transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(cone_pose_stamped, transform)
        
        
        if transformed_pose.pose.position.x > 0.0:
            self.cone_x = transformed_pose.pose.position.x 
            self.cone_y = transformed_pose.pose.position.y 
        # ✅ Now you have the pose in the map frame
        self.get_logger().info(f"Cone position : X : {self.cone_x} Y : { self.cone_y}")
				
		

    def publish_path(self):
        """ Function to generate and publish the path in the map frame """
        if self.carposition_x is None or self.carposition_y is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # Define obstacle and avoidance thresholds
        avoiding_thresholdy = 15.0
        minavoiding_thresholdx = 2.0
        maxavoiding_thresholdx = 3.0
        obstacle_x = self.cone_x
        obstacle_y = self.cone_y

        # Transform car's current full pose from map frame to car frame
        transformed_pose = self.transform_to_car_frame(self.carposition_x, self.carposition_y, self.yaw)
        self.lastpointposition_x = transformed_pose.pose.position.x
        self.lastpointposition_y = transformed_pose.pose.position.y
        last_yaw = self.quaternion_to_yaw(transformed_pose.pose.orientation)

        self.path.poses.clear()
        self.path.header.stamp = self.get_clock().now().to_msg()

        num_points = 10000
        step_y = 0.1

        for i in range(num_points):
            distance_y = self.lastpointposition_y - obstacle_y
            distance_x = obstacle_x - self.lastpointposition_x

            # Check avoidance conditions and set path generation logic
            if distance_y <= -avoiding_thresholdy:
                #self.get_logger().info("Going straight")
                pass

            elif distance_x < minavoiding_thresholdx and 0 > distance_y >= -avoiding_thresholdy:
                #self.get_logger().info("Curve initiated")
                turning_radius = 3

                x0 = self.lastpointposition_x
                y0 = self.lastpointposition_y
                
                start_pose = (x0, y0, math.pi / 2)
                inter_pose = (x0 - 1.0, y0 + 5.0, 3/4 *math.pi)
                end_pose = (x0 - 2.0, y0 + 10.0, math.pi/2)

                path1 = dubins.shortest_path(start_pose, inter_pose, turning_radius)
                path2 = dubins.shortest_path(inter_pose, end_pose, turning_radius)

                sample_distance = 0.1
                configurations1, _ = path1.sample_many(sample_distance)
                configurations2, _ = path2.sample_many(sample_distance)

                for (px, py, pyaw) in configurations1 and configurations2:
                    # Create a PoseStamped object for the path in the car's local frame (base_link)
                    car_pose = PoseStamped()
                    car_pose.header.frame_id = 'ZOE3/base_link'
                    car_pose.header.stamp = self.get_clock().now().to_msg()
                    car_pose.pose.position.x = px
                    car_pose.pose.position.y = py
               
                    car_pose.pose.position.z = 0.0
                    car_pose.pose.orientation = self.yaw_to_quaternion(pyaw)

                    # Transform the pose to the map frame
                    transformed_pose = self.transform_pose_to_map_frame(car_pose)

                    # Append the transformed pose to the path
                    self.path.poses.append(transformed_pose)
                    self.lastpointposition_x = px
                    self.lastpointposition_y = py
            elif minavoiding_thresholdx <= distance_x <= maxavoiding_thresholdx and -avoiding_thresholdy <= distance_y <= avoiding_thresholdy:
                #go straight while obstacle is on the side
                #self.get_logger().info("Going straight")
                pass

            elif 1.0 < distance_x < maxavoiding_thresholdx and distance_y > avoiding_thresholdy / 2:
                #turn right
                #self.get_logger().info("curve initated")
                turning_radius = 3
               
                x0 = self.lastpointposition_x
                y0 = self.lastpointposition_y
                
                # Define the two Dubins poses with specified yaws:
                start_pose = (x0, y0, math.pi/2)  # facing left (-90 deg)
                inter_pose = (x0 + 1.0, y0 + 5 , math.pi / 4)  # 45 deg left
                end_pose = (x0 + 2.0, y0 + 10 , math.pi/2)  # facing forward, shifted -2m left

                path1 = dubins.shortest_path(start_pose, inter_pose, turning_radius)
                path2 = dubins.shortest_path(inter_pose, end_pose, turning_radius)

                sample_distance = 0.05
                configurations1, _ = path1.sample_many(sample_distance)
                configurations2, _ = path2.sample_many(sample_distance)

        
                for (px, py, pyaw) in configurations1 and configurations2:
                    # Create a PoseStamped object for the path in the car's local frame (base_link)
                    car_pose = PoseStamped()
                    car_pose.header.frame_id = 'ZOE3/base_link'
                    car_pose.header.stamp = self.get_clock().now().to_msg()
                    car_pose.pose.position.x = px
                    car_pose.pose.position.y = py
               
                    car_pose.pose.position.z = 0.0
                    car_pose.pose.orientation = self.yaw_to_quaternion(pyaw)

                    # Transform the pose to the map frame
                    transformed_pose = self.transform_pose_to_map_frame(car_pose)

                    # Append the transformed pose to the path
                    self.path.poses.append(transformed_pose)
                    self.lastpointposition_x = px
                    self.lastpointposition_y = py
                    
            elif 0 < distance_x < 1 and distance_y > avoiding_thresholdy:
                #self.get_logger().info("Going straight")
                #Go straight after the obstacle
                pass
                
                
            # Increment along y for straight or after curve
            self.lastpointposition_y += step_y
            car_pose = PoseStamped()
            car_pose.header.frame_id = 'ZOE3/base_link'
            car_pose.header.stamp = self.get_clock().now().to_msg()
            car_pose.pose.position.x = self.lastpointposition_x
            car_pose.pose.position.y = self.lastpointposition_y
      
            car_pose.pose.orientation = self.yaw_to_quaternion(last_yaw)

            # Transform the pose to the map frame
            transformed_pose = self.transform_pose_to_map_frame(car_pose)
            self.path.poses.append(transformed_pose)

        self.publisher.publish(self.path)
        self.get_logger().info(f'Published standard path with {len(self.path.poses)} points')

    def transform_pose_to_map_frame(self, pose_in_car_frame: PoseStamped) -> PoseStamped:
        """ Transform full pose from car frame (base_link) to map frame (dummy) """
        try:
            # Get transform from base_link to dummy (map)
            transform = self.tf_buffer.lookup_transform('map', 'ZOE3/base_link', rclpy.time.Time())
            # Transform full PoseStamped
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_in_car_frame, transform)
           
            return transformed_pose
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return pose_in_car_frame  # fallback: return input pose

    def transform_to_car_frame(self, map_x, map_y, map_yaw=0.0) -> PoseStamped:
        """ Transform full pose (position + orientation) from map frame to car frame """
        try:
            # Lookup transform from dummy (map) to base_link (car frame)
            transform = self.tf_buffer.lookup_transform('ZOE3/base_link', 'map', rclpy.time.Time())

            # Create PoseStamped in map frame
            pose_in_map_frame = PoseStamped()
            pose_in_map_frame.header.frame_id = 'map'
            pose_in_map_frame.header.stamp = self.get_clock().now().to_msg()
            pose_in_map_frame.pose.position.x = map_x
            pose_in_map_frame.pose.position.y = map_y
            pose_in_map_frame.pose.position.z = 0.0
            pose_in_map_frame.pose.orientation = self.yaw_to_quaternion(map_yaw)

            # Transform full PoseStamped
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_in_map_frame, transform)

            return transformed_pose

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            fallback_pose = PoseStamped()
            fallback_pose.header.frame_id = 'ZOE3/base_link'
            fallback_pose.header.stamp = self.get_clock().now().to_msg()
            fallback_pose.pose.position.x = map_x
            fallback_pose.pose.position.y = map_y
            fallback_pose.pose.position.z = 0.0
            fallback_pose.pose.orientation = self.yaw_to_quaternion(map_yaw)
            return fallback_pose

    def yaw_to_quaternion(self, yaw) -> Quaternion:
        """ Convert yaw (radians) to quaternion orientation """
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """ Convert quaternion to yaw angle """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
