#ifndef LANE_WIDTH_CALCULATOR__NODE_HPP_
#define LANE_WIDTH_CALCULATOR__NODE_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


namespace lane_width_calculator
{

using autoware_auto_mapping_msgs::msg::HADMapBin;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

class CalculatorNode: public rclcpp::Node
{
public:
  CalculatorNode(const rclcpp::NodeOptions & options);
  ~CalculatorNode() = default;

  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  // const lanelets
  lanelet::ConstLanelets const_lanelets_;
  bool map_loaded_ = false;
  bool use_odom_;

  // subscribers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  double vehicle_width_;
  double vehicle_length_;

  std::unordered_map<std::string, geometry_msgs::msg::Pose> position_pose_map_;

// Member Functions
  void mapCallback(const HADMapBin::ConstSharedPtr msg);
  void poseCallback(const PoseStamped::ConstSharedPtr msg);
  void odomCallback(const Odometry::ConstSharedPtr msg);
  void updateVehiclePoses(const geometry_msgs::msg::Pose & pose);
};


}



#endif  // LANE_WIDTH_CALCULATOR__NODE_HPP_