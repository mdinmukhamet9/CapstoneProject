/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2025, Dimash Mukhamet
 */

 #ifndef MPC_LOW__MPC_CONTROLLER_HPP_
 #define MPC_LOW__MPC_CONTROLLER_HPP_
 
 #include <memory>
 #include <string>
 
 #include "nav2_core/controller.hpp"
 #include "nav2_util/node_utils.hpp"
 #include "nav2_costmap_2d/costmap_2d_ros.hpp"
 #include "tf2_ros/buffer.h"
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_lifecycle/lifecycle_node.hpp"
 #include "geometry_msgs/msg/pose_stamped.hpp"
 #include "geometry_msgs/msg/twist_stamped.hpp"
 #include "nav_msgs/msg/path.hpp"
 #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
 #include "acados_solver_holder.hpp"
 
 namespace mpc_low
 {
 
 class MPCController : public nav2_core::Controller
 {
 public:
     MPCController();
     ~MPCController() override = default;
 
     void configure(
         const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
         std::string name,
         std::shared_ptr<tf2_ros::Buffer> tf,
         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
 
     void cleanup() override;
     void activate() override;
     void deactivate() override;
 
     geometry_msgs::msg::TwistStamped computeVelocityCommands(
         const geometry_msgs::msg::PoseStamped & pose,
         const geometry_msgs::msg::Twist & velocity,
         nav2_core::GoalChecker * goal_checker) override;
 
     void setPlan(const nav_msgs::msg::Path & path) override;
     void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
 
 protected:
     nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
 
     template<typename Iter>
     Iter min_by(Iter begin, Iter end, std::function<double(geometry_msgs::msg::PoseStamped)> pred)
     {
         if (begin == end) {
             return end;
         }
         Iter min = begin;
         double min_val = pred(*begin);
         for (Iter it = begin; it != end; ++it) {
             double val = pred(*it);
             if (val < min_val) {
                 min_val = val;
                 min = it;
             }
         }
         return min;
     }
 
     bool transformPose(
         const std::shared_ptr<tf2_ros::Buffer> tf,
         const std::string frame,
         const geometry_msgs::msg::PoseStamped & in_pose,
         geometry_msgs::msg::PoseStamped & out_pose,
         const rclcpp::Duration & transform_tolerance);
 
     rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
     std::shared_ptr<tf2_ros::Buffer> tf_;
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
     std::string plugin_name_;
     rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
     rclcpp::Clock::SharedPtr clock_;
     rclcpp::Duration transform_tolerance_;
     nav_msgs::msg::Path global_plan_;
     rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_pub_;
 
     // NMPC solver
     std::unique_ptr<my_NMPC_solver> nmpc_solver_;
     double desired_linear_vel_;
     double max_angular_vel_;
     int nmpc_horizon_steps_;
 };
 
 }  // namespace mpc_low
 
 #endif  // MPC_LOW__MPC_CONTROLLER_HPP_