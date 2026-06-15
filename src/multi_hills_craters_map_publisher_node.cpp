#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

struct Hill
{
  double x;
  double y;
  double h;
  double w;
};

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher() : Node("multi_hills_craters_map_publisher_node")
  {
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();

    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/height_map/visualization", qos);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapPublisher::publishMap, this));

    RCLCPP_INFO(get_logger(), "Map publisher started");
  }

private:
  double gauss(double x, double y, const Hill & p)
  {
    double dx = x - p.x;
    double dy = y - p.y;
    return p.h * std::exp(-(dx * dx + dy * dy) / (2.0 * p.w * p.w));
  }

  double getHeight(double x, double y)
  {
    double z = 0.02 * std::sin(1.6 * x) * std::cos(1.4 * y);

    std::vector<Hill> hills = {
      {-1.6, -1.1,  0.45, 0.35},
      {-1.2,  1.4,  0.40, 0.32},
      {-0.3, -0.4,  0.50, 0.38},
      { 0.7,  0.9,  0.55, 0.36},
      { 1.6, -0.8,  0.42, 0.34},
      { 1.8,  1.7,  0.48, 0.35}
    };

    std::vector<Hill> craters = {
      {-1.7,  0.9, -0.42, 0.32},
      {-0.8, -1.3, -0.35, 0.30},
      { 0.5, -1.1, -0.50, 0.36},
      { 1.3,  0.2, -0.40, 0.30},
      { 0.2,  1.5, -0.32, 0.26}
    };

    for (auto & p : hills) {
      z += gauss(x, y, p);
    }

    for (auto & p : craters) {
      z += gauss(x, y, p);
    }

    return z;
  }

  void publishMap()
  {
    grid_map::GridMap map;
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(5.0, 5.0), 0.05, grid_map::Position(0.0, 0.0));
    map.add("elevation");
    map.setBasicLayers({"elevation"});

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      map.getPosition(*it, pos);
      map.at("elevation", *it) = getHeight(pos.x(), pos.y());
    }

    map.setTimestamp(now().nanoseconds());

    auto msg = grid_map::GridMapRosConverter::toMessage(map);
    map_pub_->publish(std::move(msg));

    publishMarker(map);
  }

  void publishMarker(const grid_map::GridMap & map)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();

    marker.ns = "height_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = map.getResolution();
    marker.scale.y = map.getResolution();
    marker.scale.z = 0.035;

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      double z = map.at("elevation", *it);
      min_z = std::min(min_z, z);
      max_z = std::max(max_z, z);
    }

    double range = std::max(0.001, max_z - min_z);

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      map.getPosition(*it, pos);

      double z = map.at("elevation", *it);
      double t = (z - min_z) / range;

      geometry_msgs::msg::Point point;
      point.x = pos.x();
      point.y = pos.y();
      point.z = z;

      std_msgs::msg::ColorRGBA color;
      color.r = static_cast<float>(t);
      color.g = 0.25f;
      color.b = static_cast<float>(1.0 - t);
      color.a = 0.85f;

      marker.points.push_back(point);
      marker.colors.push_back(color);
    }

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}