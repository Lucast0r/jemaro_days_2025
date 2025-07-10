#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "cone_detection.h"

namespace cone_detection
{
std::string SUBSCRIBED_TOPIC, PUBLISHED_TOPIC, PUB_CLOUD_FRAME;
float LEAF_SIZE_X, LEAF_SIZE_Y, LEAF_SIZE_Z;
bool DOWNSAMPLE_ALL_DATA;
float minx, miny, minz, val1;
float maxx, maxy, maxz, val2;

ConeDetection::ConeDetection(const rclcpp::NodeOptions & options)
: Node("cone_detection", options)
{
  RCLCPP_INFO(get_logger(), "Start ConeDetection!");

  getParams();
  rclcpp::QoS qos = rclcpp::SensorDataQoS().best_effort();
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(PUBLISHED_TOPIC, qos);
  conePosition_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/conePosition",1000);

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    SUBSCRIBED_TOPIC, qos_profile,
    std::bind(&ConeDetection::voxelFiltering, this, std::placeholders::_1));
    


  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

ConeDetection::~ConeDetection() {}

void ConeDetection::voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  pcl::PointCloud<pcl::PointXYZI> final;
  // POINT CLOUD FROM DATA
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_filtered;
  pcl::PointCloud<pcl::PointXYZI> cloud_out;

  // FILTER using crop box -- This piece works!!!
  pcl::CropBox<pcl::PointXYZI> cropFilter (true);
  cropFilter.setInputCloud(pcl_cloud.makeShared());
  Eigen::Vector4f min_pt (0.0f, -2.2f, -5.0f, 1.0f);
  Eigen::Vector4f max_pt (40.0f, 4.0f, -0.35f, 1.0f);

  cropFilter.setMax(max_pt);
  cropFilter.setMin(min_pt);
  cropFilter.filter(cloud_out);

  // RANSAC - REMOVE GROUND
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud_out.makeShared());
  seg.segment (*inliers, *coefficients);
  \
  // TAKE OUT USEFUL INFO
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_out.makeShared());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (pcl_cloud_filtered);
  // CLOUD FOR GROUND
  pcl::PointCloud<pcl::PointXYZI> groundPoints;
  extract.setNegative (false);
  extract.filter (groundPoints);

  // Voxel filtering OF GROUND
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(groundPoints.makeShared());
  sor.setLeafSize(2*LEAF_SIZE_X, 2*LEAF_SIZE_Y, LEAF_SIZE_Z);
  sor.setDownsampleAllData(DOWNSAMPLE_ALL_DATA);
  sor.filter(final);

//  RCLCPP_INFO(this->get_logger(), "Publishing: groundPoints");
//  RCLCPP_INFO(this->get_logger(), "Publishing: final size: '%f'",double(final.size()));


  pcl::PointCloud<pcl::PointXYZI> newPoints;

  std::vector<float> yValues;
//  float zSum{0};
  for (const auto& point: final.points)
  {
    yValues.push_back(point.y);
//    zSum += point.z;
//    RCLCPP_INFO(this->get_logger(), "Publishing (x,y,z): '%f', '%f', '%f' ", point._PointXYZI::x, point._PointXYZI::y, point._PointXYZI::z);

//    if (point._PointXYZI::y > 1.5 || point._PointXYZI::y < -1.0)
//    {
//      RCLCPP_INFO(this->get_logger(), "Publishing (x,y,z): '%f', '%f', '%f' ", point._PointXYZI::x, point._PointXYZI::y, point._PointXYZI::z);
//    }
  }
  std::sort(yValues.begin(), yValues.end());

  float medianY = yValues[yValues.size() / 2]; // CHECK IF THE ROAD IS TOO WIDE? (IF TOO WIDE/SKINNY DON'T CONSIDER???)
  bool publishPose{false};
//  float zAverage = zSum / yValues.size();
//  RCLCPP_INFO(this->get_logger(), "Publishing: zAverage: '%f'",zAverage);
  for (const auto& point: pcl_cloud_filtered.points)
  {
    if (point.y > (medianY-2.0f) && point.y < (medianY + 2.0f))
    {
      if (point.z > -1.5f+0.2f && point.intensity > 1000.0f)
      {
//        RCLCPP_INFO(this->get_logger(), "Publishing: POTENTIAL CONE DETECTED AT: '%f', '%f', '%f',",point.x, point.y, point.z);
//        RCLCPP_INFO(this->get_logger(), "Publishing: Intensity: '%f'",point.intensity);
        newPoints.push_back(point);
        publishPose = true;
      }
    }
  }
  bool allPointsClose = true;

//  float thresholdDistance = 1.0f;
  if (publishPose == true)
  {
    float avgX{0}, avgY{0};
    for (const auto& point: newPoints.points)
    {
      avgX += point.x;
      avgY += point.y;
    }

    avgX = avgX / newPoints.points.size();
    avgY = avgY / newPoints.points.size();
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = avgX;
    pose.pose.position.y = avgY;



    for (const auto& point : newPoints.points) {
      float distance = std::sqrt(std::pow(point.x - avgX, 2) + std::pow(point.y - avgY, 2));
      if (distance > 0.07) {
        allPointsClose = false;
        break;
      }
    }
    if (allPointsClose == true)
    {
      pose.header.stamp = this->now();
      pose.header.frame_id = "ZOE3/os_sensor";

	  try {
	    geometry_msgs::msg::PoseStamped transformed_pose = tf_buffer_->transform(
	  	  pose, "map", tf2::durationFromSec(0.5));

        if (old_x == 0.0f)
		{
			conePosition_pub_->publish(transformed_pose);
            old_x = transformed_pose.pose.position.x;
            old_y = transformed_pose.pose.position.y;
        }
		else 
		{
            float dif_x = std::fabs(double(old_x-transformed_pose.pose.position.x));
            float dif_y = std::fabs(double(old_y-transformed_pose.pose.position.y));
			if (dif_x < 1.0f || dif_y < 1.0f)
			{
			}
			else
			{
				conePosition_pub_->publish(transformed_pose);
                old_x = transformed_pose.pose.position.x;
                old_y = transformed_pose.pose.position.y;
			}
			
		}
		
	    conePosition_pub_->publish(transformed_pose);
	  } catch (const tf2::TransformException & ex) {
	    RCLCPP_WARN(this->get_logger(), "Transform to map frame failed: %s", ex.what());
	  }
    }
  }
  // Find the cone and publish cone location

  sensor_msgs::msg::PointCloud2 ros2_cloud_filtered;
//  ros2_cloud_filtered.set__data(msg->data[0]);
//  ros2_cloud_filtered = msg->data[0];CONES
//  pcl::toROSMsg(cloud_out, ros2_cloud_filtered); // This one works
//  pcl::toROSMsg(final, ros2_cloud_filtered); // This publishes the voxelized ground plane
//  pcl::toROSMsg(pcl_cloud_filtered, ros2_cloud_filtered); // This is the non ground objects
  pcl::toROSMsg(newPoints, ros2_cloud_filtered); // just for fun -- SHOWS THE CONES
  RCLCPP_DEBUG(get_logger(), "before filter: %d", msg->width);
  RCLCPP_DEBUG(get_logger(), "after filter: %d", ros2_cloud_filtered.width);

  ros2_cloud_filtered.header.frame_id = PUB_CLOUD_FRAME;
  ros2_cloud_filtered.header.stamp = get_clock()->now();
  ros2_cloud_filtered.is_dense = true;

  pointcloud_pub_->publish(std::move(ros2_cloud_filtered));
}

void ConeDetection::getParams()
{
  declare_parameter<std::string>("sub_topic", "/cloud_registered_body1");
  declare_parameter<std::string>("pub_topic", "/cloud_registered_body_downsampling1");
  declare_parameter<std::string>("pub_cloud_frame", "livox_frame1");
  declare_parameter<float>("leaf_size_x", 0.2);
  declare_parameter<float>("leaf_size_y", 0.2);
  declare_parameter<float>("leaf_size_z", 0.2);
  declare_parameter<bool>("downsample_all_data", false);

  get_parameter_or<std::string>("sub_topic", SUBSCRIBED_TOPIC, "/cloud_registered_body1");
  get_parameter_or<std::string>(
    "pub_topic", PUBLISHED_TOPIC, "/cloud_registered_body_downsampling1");
  get_parameter_or<std::string>("pub_cloud_frame", PUB_CLOUD_FRAME, "livox_frame1");
  get_parameter_or<float>("leaf_size_x", LEAF_SIZE_X, 0.2);
  get_parameter_or<float>("leaf_size_y", LEAF_SIZE_Y, 0.2);
  get_parameter_or<float>("leaf_size_z", LEAF_SIZE_Z, 0.2);
  get_parameter_or<bool>("downsample_all_data", DOWNSAMPLE_ALL_DATA, false);

  RCLCPP_INFO(this->get_logger(), "subscribed_topic %s", SUBSCRIBED_TOPIC.c_str());
  RCLCPP_INFO(this->get_logger(), "published_topic %s", PUBLISHED_TOPIC.c_str());
  RCLCPP_INFO(this->get_logger(), "leaf_size_x %f", LEAF_SIZE_X);
  RCLCPP_INFO(this->get_logger(), "leaf_size_y %f", LEAF_SIZE_Y);
  RCLCPP_INFO(this->get_logger(), "leaf_size_z %f", LEAF_SIZE_Z);
  RCLCPP_INFO(this->get_logger(), "downsample_all_data %s", DOWNSAMPLE_ALL_DATA ? "true" : "false");
}
}  // namespace cone_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cone_detection::ConeDetection)
