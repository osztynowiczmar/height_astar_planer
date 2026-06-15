#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Ustawienia programu
const std::string MAP_TOPIC = "/grid_map";
const std::string ELEVATION_LAYER = "elevation";

const std::string START_TOPIC = "/initialpose";
const std::string GOAL_TOPIC = "/goal_pose";

// false - start i cel ustawiane w RViz
// true  - start i cel brane z wartości poniżej
const bool USE_POINTS_FROM_CODE = false;

const double START_X = -1.5;
const double START_Y = -1.5;
const double GOAL_X = 1.5;
const double GOAL_Y = 1.5;

const double SLOPE_WEIGHT = 4.0;
const double MAX_SLOPE = 1.2;
const bool ALLOW_DIAGONAL = true;

class AStarPlannerNode : public rclcpp::Node
{
public:
  AStarPlannerNode() : Node("astar_planner_node")
  {
    start_x_ = START_X;
    start_y_ = START_Y;
    goal_x_ = GOAL_X;
    goal_y_ = GOAL_Y;

    start_ok_ = USE_POINTS_FROM_CODE;
    goal_ok_ = USE_POINTS_FROM_CODE;

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      MAP_TOPIC, map_qos,
      std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));

    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      START_TOPIC, 10,
      std::bind(&AStarPlannerNode::startCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      GOAL_TOPIC, 10,
      std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/astar/path", 1);
    visited_pub_ = create_publisher<visualization_msgs::msg::Marker>("/astar/explored", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/astar/path_marker", 1);
    start_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>("/astar/start_goal", 1);

    RCLCPP_INFO(get_logger(), "A* planner started");
  }

private:
  struct NodeData
  {
    int id;
    double f;

    bool operator<(const NodeData & other) const
    {
      return f > other.f;   // odwrócenie dla priority_queue
    }
  };

  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap tmp_map;

    if (!grid_map::GridMapRosConverter::fromMessage(*msg, tmp_map)) {
      RCLCPP_ERROR(get_logger(), "Cannot read grid map");
      return;
    }

    if (!tmp_map.exists(ELEVATION_LAYER)) {
      RCLCPP_ERROR(get_logger(), "No elevation layer in map");
      return;
    }

    tmp_map.convertToDefaultStartIndex();
    map_ = std::make_unique<grid_map::GridMap>(tmp_map);
    map_ok_ = true;

    if (!planned_) {
      plan();
    }
  }

  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;
    start_ok_ = true;
    planned_ = false;

    RCLCPP_INFO(get_logger(), "Start: %.2f %.2f", start_x_, start_y_);
    plan();
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_ok_ = true;
    planned_ = false;

    RCLCPP_INFO(get_logger(), "Goal: %.2f %.2f", goal_x_, goal_y_);
    plan();
  }

  void plan()
  {
    if (!map_ok_ || !start_ok_ || !goal_ok_) {
      RCLCPP_INFO(get_logger(), "Waiting for map/start/goal...");
      return;
    }

    grid_map::Index start, goal;

    if (!map_->getIndex(grid_map::Position(start_x_, start_y_), start) ||
        !map_->getIndex(grid_map::Position(goal_x_, goal_y_), goal)) {
      RCLCPP_ERROR(get_logger(), "Start or goal is outside the map");
      return;
    }

    if (!map_->isValid(start, ELEVATION_LAYER) || !map_->isValid(goal, ELEVATION_LAYER)) {
      RCLCPP_ERROR(get_logger(), "Start or goal cell is invalid");
      return;
    }

    publishStartGoal();

    std::vector<grid_map::Index> path;
    std::vector<grid_map::Index> visited;
    double cost = 0.0;

    auto t1 = std::chrono::steady_clock::now();
    bool found = runAStar(start, goal, path, visited, cost);
    auto t2 = std::chrono::steady_clock::now();

    double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

    publishVisited(visited);

    if (!found) {
      clearPath();
      RCLCPP_WARN(get_logger(), "Path not found, visited: %zu, time: %.2f ms", visited.size(), time_ms);
      return;
    }

    publishPath(path);
    publishPathMarker(path);
    planned_ = true;

    RCLCPP_INFO(
      get_logger(), "Path found, cells: %zu, visited: %zu, cost: %.3f, time: %.2f ms",
      path.size(), visited.size(), cost, time_ms);
  }

  bool runAStar(
    const grid_map::Index & start,
    const grid_map::Index & goal,
    std::vector<grid_map::Index> & path,
    std::vector<grid_map::Index> & visited,
    double & final_cost)
  {
    int rows = map_->getSize()(0);
    int cols = map_->getSize()(1);
    int n = rows * cols;

    int start_id = start(0) * cols + start(1);
    int goal_id = goal(0) * cols + goal(1);

    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<bool> closed(n, false);

    std::priority_queue<NodeData> open;

    g[start_id] = 0.0;
    parent[start_id] = start_id;
    open.push({start_id, heuristic(start, goal)});

    const int dirs[8][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1},
      {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    int dirs_count = ALLOW_DIAGONAL ? 8 : 4;

    while (!open.empty()) {
      int id = open.top().id;
      open.pop();

      if (closed[id]) {
        continue;
      }

      closed[id] = true;

      grid_map::Index current;
      current << id / cols, id % cols;
      visited.push_back(current);

      if (id == goal_id) {
        final_cost = g[id];
        recreatePath(parent, goal_id, cols, path);
        return true;
      }

      for (int i = 0; i < dirs_count; i++) {
        int r = current(0) + dirs[i][0];
        int c = current(1) + dirs[i][1];

        if (r < 0 || r >= rows || c < 0 || c >= cols) {
          continue;
        }

        grid_map::Index next;
        next << r, c;

        if (!map_->isValid(next, ELEVATION_LAYER)) {
          continue;
        }

        int next_id = r * cols + c;

        if (closed[next_id]) {
          continue;
        }

        double move_cost = costBetween(current, next);

        if (!std::isfinite(move_cost)) {
          continue;
        }

        double new_g = g[id] + move_cost;

        if (new_g < g[next_id]) {
          g[next_id] = new_g;
          parent[next_id] = id;
          open.push({next_id, new_g + heuristic(next, goal)});
        }
      }
    }

    return false;
  }

  void recreatePath(
    const std::vector<int> & parent,
    int goal_id,
    int cols,
    std::vector<grid_map::Index> & path)
  {
    int id = goal_id;

    while (id >= 0) {
      grid_map::Index index;
      index << id / cols, id % cols;
      path.push_back(index);

      if (parent[id] == id) {
        break;
      }

      id = parent[id];
    }

    std::reverse(path.begin(), path.end());
  }

  double costBetween(const grid_map::Index & a, const grid_map::Index & b)
  {
    double dx = static_cast<double>(b(0) - a(0));
    double dy = static_cast<double>(b(1) - a(1));
    double dist = std::hypot(dx, dy) * map_->getResolution();

    double h1 = map_->at(ELEVATION_LAYER, a);
    double h2 = map_->at(ELEVATION_LAYER, b);
    double slope = std::abs(h2 - h1) / dist;

    if (MAX_SLOPE > 0.0 && slope > MAX_SLOPE) {
      return std::numeric_limits<double>::infinity();
    }

    return dist * (1.0 + SLOPE_WEIGHT * slope);
  }

  double heuristic(const grid_map::Index & a, const grid_map::Index & b)
  {
    double dx = static_cast<double>(a(0) - b(0));
    double dy = static_cast<double>(a(1) - b(1));
    return std::hypot(dx, dy) * map_->getResolution();
  }

  geometry_msgs::msg::Point indexToPoint(const grid_map::Index & index, double offset)
  {
    grid_map::Position pos;
    map_->getPosition(index, pos);

    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = map_->at(ELEVATION_LAYER, index) + offset;
    return p;
  }

  double getHeight(double x, double y)
  {
    grid_map::Index index;
    if (!map_->getIndex(grid_map::Position(x, y), index)) {
      return 0.0;
    }
    if (!map_->isValid(index, ELEVATION_LAYER)) {
      return 0.0;
    }
    return map_->at(ELEVATION_LAYER, index);
  }

  void publishPath(const std::vector<grid_map::Index> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    for (const auto & index : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = indexToPoint(index, 0.08);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
  }

  void publishPathMarker(const std::vector<grid_map::Index> & path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "astar_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    for (const auto & index : path) {
      marker.points.push_back(indexToPoint(index, 0.12));
    }

    marker_pub_->publish(marker);
  }

  void publishVisited(const std::vector<grid_map::Index> & visited)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "astar_visited";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_->getResolution() * 0.35;
    marker.scale.y = map_->getResolution() * 0.35;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.a = 0.75f;

    for (const auto & index : visited) {
      marker.points.push_back(indexToPoint(index, 0.04));
    }

    visited_pub_->publish(marker);
  }

  void publishStartGoal()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "astar_start_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    geometry_msgs::msg::Point start;
    start.x = start_x_;
    start.y = start_y_;
    start.z = getHeight(start_x_, start_y_) + 0.18;

    geometry_msgs::msg::Point goal;
    goal.x = goal_x_;
    goal.y = goal_y_;
    goal.z = getHeight(goal_x_, goal_y_) + 0.18;

    std_msgs::msg::ColorRGBA green;
    green.g = 1.0f;
    green.a = 1.0f;

    std_msgs::msg::ColorRGBA red;
    red.r = 1.0f;
    red.a = 1.0f;

    marker.points.push_back(start);
    marker.colors.push_back(green);
    marker.points.push_back(goal);
    marker.colors.push_back(red);

    start_goal_pub_->publish(marker);
  }

  void clearPath()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "astar_path";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);

    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = now();
    path_pub_->publish(empty_path);
  }

  double start_x_;
  double start_y_;
  double goal_x_;
  double goal_y_;

  bool map_ok_ = false;
  bool start_ok_ = false;
  bool goal_ok_ = false;
  bool planned_ = false;

  std::unique_ptr<grid_map::GridMap> map_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visited_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_goal_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}