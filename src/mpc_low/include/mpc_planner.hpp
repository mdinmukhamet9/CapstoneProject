#ifndef NAV2_MPC_PLANNER__MPC_PLANNER_HPP_
#define NAV2_MPC_PLANNER__MPC_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "acados_solver_holder.hpp"

namespace nav2_mpc_planner
{

class MPCPlanner : public nav2_core::GlobalPlanner
{
public:
  MPCPlanner() = default;
  ~MPCPlanner() override = default;

  // Plugin lifecycle
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Main planning interface
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Helper methods
  std::array<double, 520> process_costmap() const;
  std::array<double, 30> create_tracking_goal(
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseStamped & start) const;
  
  void update_path_from_trajectory(
    const double trajectory[33],
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start) const;
  
  void handle_solver_status(int status);

  // ROS2 components
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
  std::string name_;

  // MPC Solver
  std::unique_ptr<my_NMPC_solver> myMpcSolver;


  // Configuration
  double control_rate_;
  int mpc_horizon_;
  double interpolation_resolution_;
};

}  // namespace nav2_mpc_planner

#endif  // NAV2_MPC_PLANNER__MPC_PLANNER_HPP_