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

using namespace std::chrono_literals;

// Prosty publisher mapy wysokościowej z labiryntem.
// Mapa jest wysyłana na /grid_map, a wizualizacja na /height_map/visualization.

struct Hill
{
  double x;
  double y;
  double height;
  double width;
};

struct Wall
{
  bool horizontal;
  double pos;
  double from;
  double to;
};

class LabyrinthMapPublisherNode : public rclcpp::Node
{
public:
  LabyrinthMapPublisherNode() : Node("labyrinth_map_publisher_node")
  {
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();

    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/height_map/visualization", qos);

    timer_ = create_wall_timer(1s, std::bind(&LabyrinthMapPublisherNode::publishMap, this));

    RCLCPP_INFO(get_logger(), "Labyrinth map publisher started");
  }

private:
  double gaussian(double x, double y, const Hill & h)
  {
    double dx = x - h.x;
    double dy = y - h.y;
    double r2 = dx * dx + dy * dy;

    return h.height * std::exp(-r2 / (2.0 * h.width * h.width));
  }

  bool isWall(double x, double y)
  {
    const double wall_width = 0.075;

    // horizontal = true  -> ściana pozioma, pos oznacza y
    // horizontal = false -> ściana pionowa, pos oznacza x
    std::vector<Wall> walls = {
      {true,  -2.30, -2.30,  1.70},
      {true,   2.30, -1.60,  2.30},
      {false, -2.30, -2.30,  2.30},
      {false,  2.30, -2.30,  2.30},

      {true,  -1.55, -2.30,  1.15},
      {false,  1.15, -1.55,  0.45},
      {true,   0.45, -1.35,  2.30},
      {false, -1.35, -0.85,  1.75},
      {true,   1.75, -1.35,  1.25},
      {false,  0.15, -2.30, -0.45},
      {true,  -0.45, -0.95,  1.35},
      {false,  1.75, -0.15,  1.45},
      {true,  -1.05,  0.65,  2.30},
      {false, -1.85,  0.35,  2.30}
    };

    for (auto wall : walls) {
      if (wall.horizontal) {
        if (std::abs(y - wall.pos) < wall_width && x >= wall.from && x <= wall.to) {
          return true;
        }
      } else {
        if (std::abs(x - wall.pos) < wall_width && y >= wall.from && y <= wall.to) {
          return true;
        }
      }
    }

    return false;
  }

  double terrain(double x, double y)
  {
    double z = 0.015 * std::sin(1.6 * x) * std::cos(1.4 * y);

    std::vector<Hill> hills = {
      {-1.6, -1.1,  0.30, 0.35},
      {-1.2,  1.4,  0.28, 0.32},
      {-0.3, -0.4,  0.32, 0.38},
      { 0.7,  0.9,  0.35, 0.36},
      { 1.6, -0.8,  0.28, 0.34},
      { 1.8,  1.7,  0.30, 0.35}
    };

    std::vector<Hill> holes = {
      {-1.7,  0.9, -0.25, 0.32},
      {-0.8, -1.3, -0.22, 0.30},
      { 0.5, -1.1, -0.30, 0.36},
      { 1.3,  0.2, -0.24, 0.30},
      { 0.2,  1.5, -0.22, 0.26}
    };

    for (auto h : hills) {
      z += gaussian(x, y, h);
    }

    for (auto h : holes) {
      z += gaussian(x, y, h);
    }

    if (isWall(x, y)) {
      z -= 0.4;   // ściana jako duży spadek/wysoki koszt dla A*
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

      map.at("elevation", *it) = terrain(pos.x(), pos.y());
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
    marker.ns = "labyrinth_map";
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

      double x = pos.x();
      double y = pos.y();
      double z = map.at("elevation", *it);

      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;

      std_msgs::msg::ColorRGBA color;
      color.a = 0.85f;

      if (isWall(x, y)) {
        color.r = 0.35f;
        color.g = 0.35f;
        color.b = 0.35f;
        color.a = 0.95f;
      } else {
        double t = (z - min_z) / range;
        color.r = static_cast<float>(t);
        color.g = 0.25f;
        color.b = static_cast<float>(1.0 - t);
      }

      marker.points.push_back(p);
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
  rclcpp::spin(std::make_shared<LabyrinthMapPublisherNode>());
  rclcpp::shutdown();

  return 0;
}