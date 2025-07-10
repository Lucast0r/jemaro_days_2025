#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace cone_detection
{
class ConeDetection : public rclcpp::Node
{
public:
  explicit ConeDetection(const rclcpp::NodeOptions & options);

  ~ConeDetection() override;

private:
  void getParams();

  void voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr conePosition_pub_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  float old_x{0}, old_y{0};
};
}  // namespace cone_detection
