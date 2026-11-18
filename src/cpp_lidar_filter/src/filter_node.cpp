#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/exceptions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "visualization_msgs/msg/marker.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode() : Node("lidar_filter_node")
  {
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/livox/lidar_filtered");
    // 裁剪参数 (CropBox)
    this->declare_parameter("min_x", -0.4);
    this->declare_parameter("max_x", 0.4);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    // 全局高度过滤范围，独立于车身 CropBox。
    this->declare_parameter("min_height", -10.0);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("height_frame", "base_link");
    this->declare_parameter("negative", true); // true = 挖掉中间
    // 降采样参数
    this->declare_parameter("leaf_size", 0.05);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          output_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
     std::chrono::milliseconds(1000), std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void publish_marker() {
    double min_x = this->get_parameter("min_x").as_double();
    double max_x = this->get_parameter("max_x").as_double();
    double min_y = this->get_parameter("min_y").as_double();
    double max_y = this->get_parameter("max_y").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double max_z = this->get_parameter("max_z").as_double();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 计算中心点位置
    marker.pose.position.x = (max_x + min_x) / 2.0;
    marker.pose.position.y = (max_y + min_y) / 2.0;
    marker.pose.position.z = (max_z + min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    // 计算尺寸
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = max_z - min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4; //半透明

    marker_pub_->publish(marker);
  }
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const std::string height_frame = this->get_parameter("height_frame").as_string();
    sensor_msgs::msg::PointCloud2 cloud_in_height_frame;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        height_frame, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*msg, cloud_in_height_frame, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform cloud from %s to %s for height filtering: %s",
        msg->header.frame_id.c_str(), height_frame.c_str(), ex.what());
      return;
    }

    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud_in_height_frame, *cloud_in);

    // 先按高度过滤，避免 negative=true 的车身 CropBox 保留高处点云。
    pcl::PassThrough<pcl::PCLPointCloud2> height_filter;
    height_filter.setInputCloud(cloud_in);
    height_filter.setFilterFieldName("z");
    height_filter.setFilterLimits(
      this->get_parameter("min_height").as_double(),
      this->get_parameter("max_height").as_double());

    pcl::PCLPointCloud2::Ptr height_filtered(new pcl::PCLPointCloud2);
    height_filter.filter(*height_filtered);

    // CropBox裁剪 (去除车身)
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(height_filtered);

    // 设置裁剪盒子的范围
    Eigen::Vector4f min_pt, max_pt;
    min_pt << this->get_parameter("min_x").as_double(),
              this->get_parameter("min_y").as_double(),
              this->get_parameter("min_z").as_double(), 1.0;
    max_pt << this->get_parameter("max_x").as_double(),
              this->get_parameter("max_y").as_double(),
              this->get_parameter("max_z").as_double(), 1.0;

    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(this->get_parameter("negative").as_bool());

    pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);
    crop.filter(*cloud_cropped);

    // //降采样 (稀疏化)
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(cloud_cropped);
    // float leaf_size = this->get_parameter("leaf_size").as_double();
    // sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    //
    // pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    // sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 filtered_height_cloud;
    pcl_conversions::fromPCL(*cloud_cropped, filtered_height_cloud);
    filtered_height_cloud.header = cloud_in_height_frame.header;

    sensor_msgs::msg::PointCloud2 output;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        msg->header.frame_id, height_frame, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(filtered_height_cloud, output, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform filtered cloud from %s back to %s: %s",
        height_frame.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }
    output.header = msg->header;

    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}