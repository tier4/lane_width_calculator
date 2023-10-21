
#include "lane_width_calculator/node.hpp"


namespace lane_width_calculator
{

// return the lateral offset from the boundary line
// value is positive if the vehicle is on the right side of the boundary line
double calcLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return motion_utils::calcLateralOffset(boundary_path, search_pose.position);
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

std::array<double,2> calcLeftOrRightOffsets(lanelet::ConstLanelet lanelet, const geometry_msgs::msg::Pose & search_pose)
{
  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_offset = calcLateralOffset(left_bound, search_pose);
  const double right_offset = calcLateralOffset(right_bound, search_pose);
  return {left_offset, right_offset};
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

  pub_debug_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
  // parameters
  vehicle_width_ = this->declare_parameter<double>("vehicle_width");
  vehicle_length_ = this->declare_parameter<double>("vehicle_length");
  vehicle_height_ = this->declare_parameter<double>("vehicle_height");

  // if set to true, use odometry instead of pose
  use_odom_ = this->declare_parameter<bool>("use_odom");

  // file logging parameters
  save_to_csv_ = this->declare_parameter<bool>("save_to_csv");
  save_file_name_ = this->declare_parameter<std::string>("save_file_name");

  // initialize csv file
  position_list_ = {"center", "front_left", "front_right", "rear_left", "rear_right"};
  if(save_to_csv_){
    file_stream_.open(save_file_name_);
    file_stream_ << "time, vehicle_x, vehicle_y, ";
    for(auto & position_name: position_list_){
      file_stream_ << position_name << "_left_offset, " << position_name << "_right_offset, ";
    }
    file_stream_ << "is_inside_lane" << std::endl;
  }
}

// deconstructor
CalculatorNode::~CalculatorNode()
{
  // close csv file
  if(save_to_csv_){
    file_stream_.close();
  }
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
    // calc left/right lateral offsets for each pose
    for(auto & pose_pair: position_pose_map_){
      position_offset_map_[pose_pair.first] = calcLeftOrRightOffsets(current_lanelets.front(), pose_pair.second);
    }
    // publish visualization marker
    publishBBOX();

    // save to csv
    if(save_to_csv_){
      appendToCSV();
    }
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


bool CalculatorNode::vehicleIsInsideLane()
{
  for (auto & pose_pair : position_offset_map_) {
    const auto & offset_pair = pose_pair.second;
    const auto left_offset = offset_pair[0];
    const auto right_offset = offset_pair[1];
    if (left_offset > 0.0) {
      std::cout << "vehicle is outside of lane" << std::endl;
      std::cout << "left offset of " << pose_pair.first << " is " << left_offset  << " [m] overed"<< std::endl;
      return false;
    }else if(right_offset < 0.0){
      std::cout << "vehicle is outside of lane" << std::endl;
      std::cout << "right offset of " << pose_pair.first << " is " << std::abs(right_offset)  << " [m] overed"<< std::endl;
      return false;
    }
  }
  return true;
}

void CalculatorNode::publishBBOX()
{
  const bool is_inside = vehicleIsInsideLane();

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map"; // frame id
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // define marker position
  marker.pose = position_pose_map_["center"];

  // set merker size
  marker.scale.x = vehicle_length_;
  marker.scale.y = vehicle_width_;
  marker.scale.z = vehicle_height_;
  
  // set marker color
  if(is_inside){
    setColor(marker, ColorSetting::GREEN);
  }else{
    setColor(marker, ColorSetting::RED);
  }
  
  // push
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  // publish
  pub_debug_markers_->publish(marker_array);
}

bool CalculatorNode::appendToCSV()
{
  const auto current_time = this->now();
  const double current_time_sec = current_time.seconds();
  const auto current_pose = position_pose_map_["center"];
  const auto vehicle_x = current_pose.position.x;
  const auto vehicle_y = current_pose.position.y;
  // 1. write time, vehicle_x, vehicle_y to csv 
  file_stream_ << std::fixed << std::setprecision(6);
  file_stream_ << current_time_sec << "," << vehicle_x << "," << vehicle_y << "," ;

  // 2. for each keys in position_list_ get left and right offset
  for(auto & position_name: position_list_){
    const auto & offset_pair = position_offset_map_.at(position_name);
    const auto left_offset = offset_pair[0];
    const auto right_offset = offset_pair[1];
    file_stream_ << left_offset << "," << right_offset << ",";
  }

  // 3. get if the vehicle is inside lane
  const bool is_inside = vehicleIsInsideLane();
  const int is_inside_int = is_inside ? 1 : 0;
  file_stream_ << is_inside_int << std::endl;
  return true;
}

} // namespace lane_width_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lane_width_calculator::CalculatorNode)