
#include "lane_width_calculator/node.hpp"


namespace lane_width_calculator
{

double calcRightLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return std::fabs(motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

double calcLeftLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(boundary_line, search_pose);
}

geometry_msgs::msg::Pose calcPoseFromRelativeOffset(
  const geometry_msgs::msg::Pose & base_pose, const double longitudinal_offset, const double lateral_offset)
{
  const double yaw = tf2::getYaw(base_pose.orientation);
  const double x = base_pose.position.x + longitudinal_offset * std::cos(yaw) -
                   lateral_offset * std::sin(yaw);
  const double y = base_pose.position.y + longitudinal_offset * std::sin(yaw) +
                   lateral_offset * std::cos(yaw);
  geometry_msgs::msg::Pose output_pose;
  output_pose = base_pose;
  output_pose.position.x = x;
  output_pose.position.y = y;
  return output_pose;
}

void calcLeftOrRightOffsets(lanelet::ConstLanelet lanelet, const geometry_msgs::msg::Pose & search_pose)
{
  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_offset = calcLeftLateralOffset(left_bound, search_pose);
  const double right_offset = calcRightLateralOffset(right_bound, search_pose);
  std::cout << "left_offset: " << left_offset << std::endl;
  std::cout << "right_offset: " << right_offset << std::endl;
}


CalculatorNode::CalculatorNode(const rclcpp::NodeOptions & options)
:Node("lane_width_calculator", options)
{
  sub_objects_ = this->create_subscription<PoseStamped>(
    "~/input/pose", 1,
    std::bind(&CalculatorNode::poseCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CalculatorNode::mapCallback, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<Odometry>(
    "~/input/odom", 1,
    std::bind(&CalculatorNode::odomCallback, this, std::placeholders::_1));

  // parameters
  vehicle_width_ = this->declare_parameter("vehicle_width", 2.5);
  vehicle_length_ = this->declare_parameter("vehicle_length", 5.0);

  // if set to true, use odometry instead of pose
  use_odom_ = this->declare_parameter("use_odom", true);
}



void CalculatorNode::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[lane width calculator]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[lane width calculator]: Map is loaded");

  const_lanelets_.clear();
  for (auto l : lanelet_map_ptr_->laneletLayer) {
    const_lanelets_.push_back(l);
  }
  map_loaded_ = true;
}


void CalculatorNode::poseCallback(const PoseStamped::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose;
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  lanelet::ConstLanelets current_lanelets;
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, query_pose, &current_lanelets)) {
    // do something
    updateVehiclePoses(query_pose);

    calcLeftOrRightOffsets(current_lanelets.front(), query_pose);
  }
}

void CalculatorNode::odomCallback(const Odometry::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||!use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose.pose;
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  lanelet::ConstLanelets current_lanelets;
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, query_pose, &current_lanelets)) {
    // do something
    updateVehiclePoses(query_pose);

    calcLeftOrRightOffsets(current_lanelets.front(), query_pose);
  }
}


  void CalculatorNode::updateVehiclePoses(const geometry_msgs::msg::Pose & pose)
{
  position_pose_map_["center"] = pose;
  position_pose_map_["front_left"] = calcPoseFromRelativeOffset(
    pose, vehicle_length_ / 2.0, vehicle_width_ / 2.0);
  position_pose_map_["front_right"] = calcPoseFromRelativeOffset(
    pose, vehicle_length_ / 2.0, -vehicle_width_ / 2.0);
  position_pose_map_["rear_left"] = calcPoseFromRelativeOffset(
    pose, -vehicle_length_ / 2.0, vehicle_width_ / 2.0);
  position_pose_map_["rear_right"] = calcPoseFromRelativeOffset(
    pose, -vehicle_length_ / 2.0, -vehicle_width_ / 2.0);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lane_width_calculator::CalculatorNode)