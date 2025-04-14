#include "mpc_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_mpc_planner
{

void MPCPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".control_rate", rclcpp::ParameterValue(40.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".mpc_horizon", rclcpp::ParameterValue(5));

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".control_rate", control_rate_);
  node_->get_parameter(name_ + ".mpc_horizon", mpc_horizon_);

  // Initialize MPC solver
  myMpcSolver = std::make_unique<my_NMPC_solver>(mpc_horizon_);


  RCLCPP_INFO(
    node_->get_logger(), "Configured MPC planner plugin %s with horizon %d",
    name_.c_str(), mpc_horizon_);
}

void MPCPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up MPC planner plugin %s",
    name_.c_str());
  node_.reset();
  tf_.reset();
  costmap_ = nullptr;
}

void MPCPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating MPC planner plugin %s",
    name_.c_str());
}

void MPCPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating MPC planner plugin %s",
    name_.c_str());
}

nav_msgs::msg::Path MPCPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Check frame_id
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  // Process costmap data
  std::array<double, 520> transformed_costmap = process_costmap();
  std::array<double, 30> tracking_goal = create_tracking_goal(goal, start);

  // Get current robot state
  tf2::Quaternion q(
    start.pose.orientation.x,
    start.pose.orientation.y,
    start.pose.orientation.z,
    start.pose.orientation.w);
  double yaw = tf2::getYaw(q);
  std::array<double, 3> current_robot_position = {
    start.pose.position.x,
    start.pose.position.y,
    yaw
  };

  // Calculate relative goal
  std::array<double, 3> current_robot_goal = {
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y,
    tf2::getYaw(goal.pose.orientation)
  };
  // // DEBUG: Print the goal being sent to the solver
  // RCLCPP_INFO(
  //   node_->get_logger(), 
  //   "MPC Goal (relative to start): (%.2f, %.2f, %.2f)", 
  //   current_robot_goal[0], 
  //   current_robot_goal[1], 
  //   current_robot_goal[2]
  // );
  // Solve MPC
  double result[8] = {0};
  double trajectory[33] = {0};
  int status = myMpcSolver->solve_my_mpc(
    current_robot_position.data(),
    transformed_costmap.data(),
    current_robot_goal.data(),
    tracking_goal.data(),
    result,
    trajectory
  );

  if (status > 0) {
    RCLCPP_WARN(node_->get_logger(), "MPC solver failed with status: %d", status);
    handle_solver_status(status);
    return global_path;
  }
  // Validate trajectory data
  bool is_trajectory_valid = true;
  for (int i = 0; i < 33; ++i) {
      if (!std::isfinite(trajectory[i])) {
          is_trajectory_valid = false;
          break;
      }
  }

  if (!is_trajectory_valid) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid trajectory data (NaN/Inf detected)");
      return nav_msgs::msg::Path(); // Return empty path
  }
  // Create path from MPC solution
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  update_path_from_trajectory(trajectory, global_path, start);

  return global_path;
}

std::array<double, 520> MPCPlanner::process_costmap() const
{
  std::array<double, 520> transformed;
  transformed.fill(-100.0); // Initialize with invalid values

  unsigned int obst_count = 0;
  for (unsigned int i = 0; i < costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(); ++i) {
    if (costmap_->getCost(i) > 99) { // Obstacle threshold
      unsigned int x = i % costmap_->getSizeInCellsX();
      unsigned int y = i / costmap_->getSizeInCellsX();
      
      double wx, wy;
      costmap_->mapToWorld(x, y, wx, wy);
      
      if (obst_count * 2 + 1 < transformed.size()) {
        transformed[2*obst_count] = wx;
        transformed[2*obst_count+1] = wy;
        obst_count++;
      } else {
        break;
      }
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Processed %u obstacles from costmap", obst_count);
  return transformed;
}

std::array<double, 30> MPCPlanner::create_tracking_goal(
  const geometry_msgs::msg::PoseStamped & goal,
  const geometry_msgs::msg::PoseStamped & start) const
{
  std::array<double, 30> tracking_goal;
  double goal_yaw = tf2::getYaw(goal.pose.orientation);

  for (int i = 0; i < 10; ++i) {
    tracking_goal[3*i] = goal.pose.position.x - start.pose.position.x;
    tracking_goal[3*i+1] = goal.pose.position.y - start.pose.position.y;
    tracking_goal[3*i+2] = goal_yaw;
  }

  return tracking_goal;
}

void MPCPlanner::update_path_from_trajectory(
  const double trajectory[33],
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start) const
{
  path.poses.clear();
  rclcpp::Time now = node_->now();
  double time_step = 0.1; // Seconds between points

  for (int i = 0; i < 11; ++i) {
    if (3*i+2 >= 33) break; // Safety check

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now + rclcpp::Duration::from_seconds(i * time_step);
    pose.header.frame_id = global_frame_;
    
    // Transform from relative to global coordinates
    pose.pose.position.x = trajectory[3*i] + start.pose.position.x;
    pose.pose.position.y = trajectory[3*i+1] + start.pose.position.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, trajectory[3*i+2]);
    pose.pose.orientation = tf2::toMsg(q);
    
    path.poses.push_back(pose);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Generated path with %zu points", path.poses.size());
}

void MPCPlanner::handle_solver_status(int status)
{
  if (status > 0) {
    RCLCPP_WARN(node_->get_logger(), "Resetting MPC solver due to status: %d", status);
    myMpcSolver->reset_solver();
    myMpcSolver = std::make_unique<my_NMPC_solver>(mpc_horizon_);
  }
}

}  // namespace nav2_mpc_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mpc_planner::MPCPlanner, nav2_core::GlobalPlanner)