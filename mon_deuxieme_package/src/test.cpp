#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>
#include <vector>

class PointCloudToScanNode : public rclcpp::Node
{
public:
  PointCloudToScanNode() : Node("pointcloud_to_scan_node")
  {
    this->declare_parameter("min_height", -0.1);
    this->declare_parameter("max_height", 0.5);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("range_min", 0.2);
    this->declare_parameter("range_max", 20.0);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10,
      std::bind(&PointCloudToScanNode::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(), "Noeud de projection C++ pret : ecoute /rslidar_points -> publie /scan");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    double min_h = this->get_parameter("min_height").as_double();
    double max_h = this->get_parameter("max_height").as_double();
    double angle_min = this->get_parameter("angle_min").as_double();
    double angle_max = this->get_parameter("angle_max").as_double();
    double range_min = this->get_parameter("range_min").as_double();
    double range_max = this->get_parameter("range_max").as_double();

    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = "rslidar";

    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = 0.0087;
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = range_min;
    scan.range_max = range_max;

    uint32_t num_readings = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || z < min_h || z > max_h)
        continue;

      float range = std::hypot(x, y);
      if (range < range_min || range > range_max)
        continue;

      float angle = std::atan2(y, x);
      if (angle < angle_min || angle > angle_max)
        continue;

      int index = (angle - angle_min) / scan.angle_increment;
      if (index >= 0 && index < static_cast<int>(num_readings))
      {
        if (range < scan.ranges[index])
          scan.ranges[index] = range;
      }
    }

    pub_->publish(scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToScanNode>());
  rclcpp::shutdown();
  return 0;
}
