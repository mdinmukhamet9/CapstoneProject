/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2025, Dimash Mukhamet
 */

 #include "mpc_controller.hpp"
 #include "nav2_util/geometry_utils.hpp"
 #include "nav2_util/node_utils.hpp"
 #include "nav2_core/exceptions.hpp"
 #include <stdexcept>
 
 using std::hypot;
 using std::min;
 using std::max;
 using std::abs;
 using nav2_util::declare_parameter_if_not_declared;
 using nav2_util::geometry_utils::euclidean_distance;
 using nav2_util::geometry_utils::min_by;
 
 namespace mpc_low
 {
 
 MPCController::MPCController()
 : transform_tolerance_(rclcpp::Duration::from_seconds(0.0)),
   desired_linear_vel_(0.5),
   max_angular_vel_(1.0),
   nmpc_horizon_steps_(10)
 {
 }
 
 void MPCController::configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name,
     const std::shared_ptr<tf2_ros::Buffer> tf,
     const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
 {
     node_ = parent;
     auto node = node_.lock();
     if (!node) {
         throw std::runtime_error("Failed to lock node");
     }
 
     costmap_ros_ = costmap_ros;
     tf_ = tf;
     plugin_name_ = name;
     logger_ = node->get_logger();
     clock_ = node->get_clock();
 
     // Declare parameters
     declare_parameter_if_not_declared(
         node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
     declare_parameter_if_not_declared(
         node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
     declare_parameter_if_not_declared(
         node, plugin_name_ + ".nmpc_horizon_steps", rclcpp::ParameterValue(10));
     declare_parameter_if_not_declared(
         node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
 
     // Get parameters
     node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
     node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
     node->get_parameter(plugin_name_ + ".nmpc_horizon_steps", nmpc_horizon_steps_);
     double transform_tolerance;
     node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
     transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
 
     RCLCPP_INFO(logger_, "Parameters: desired_linear_vel=%f, max_angular_vel=%f, nmpc_horizon_steps=%d",
         desired_linear_vel_, max_angular_vel_, nmpc_horizon_steps_);
 
     // Initialize NMPC solver
     nmpc_solver_ = std::make_unique<my_NMPC_solver>(nmpc_horizon_steps_);
 
     // Initialize publisher
     global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
 }
 
 void MPCController::cleanup()
 {
     RCLCPP_INFO(
         logger_,
         "Cleaning up controller: %s of type mpc_low::MPCController",
         plugin_name_.c_str());
     nmpc_solver_.reset();
     global_pub_.reset();
 }
 
 void MPCController::activate()
 {
     RCLCPP_INFO(
         logger_,
         "Activating controller: %s of type mpc_low::MPCController",
         plugin_name_.c_str());
     // No on_activate needed for standard publisher
 }
 
 void MPCController::deactivate()
 {
     RCLCPP_INFO(
         logger_,
         "Deactivating controller: %s of type mpc_low::MPCController",
         plugin_name_.c_str());
     // No on_deactivate needed for standard publisher
 }
 
 void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
 {
     if (percentage) {
         desired_linear_vel_ *= (speed_limit / 100.0);
         max_angular_vel_ *= (speed_limit / 100.0);
     } else {
         desired_linear_vel_ = speed_limit;
         max_angular_vel_ = speed_limit;
     }
     RCLCPP_INFO(logger_, "Updated speed limits: desired_linear_vel=%f, max_angular_vel=%f",
         desired_linear_vel_, max_angular_vel_);
 }
 
 geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::Twist & velocity,
     nav2_core::GoalChecker * goal_checker)
 {
     (void)velocity;
     (void)goal_checker;
 
     // Transform global plan to robot's frame
     auto transformed_plan = transformGlobalPlan(pose);
 
     // Prepare NMPC inputs
     double current_position[3] = {
         pose.pose.position.x,
         pose.pose.position.y,
         tf2::getYaw(pose.pose.orientation)
     };
 
     // Extract goal pose
     auto goal_pose = transformed_plan.poses.back();
     double current_goal[3] = {
         goal_pose.pose.position.x,
         goal_pose.pose.position.y,
         tf2::getYaw(goal_pose.pose.orientation)
     };
 
     // Prepare costmap data
     nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
     unsigned int mx = costmap->getSizeInCellsX();
     unsigned int my = costmap->getSizeInCellsY();
     RCLCPP_DEBUG(logger_, "Costmap size: %u x %u", mx, my);  // Should be 83x83 for 5m x 5m at 0.06m resolution
     double costmap_data[520] = {0.0};
     int index = 0;
     for (unsigned int i = 0; i < mx && i < 26 && index < 520; ++i) {
         for (unsigned int j = 0; j < my && j < 20 && index < 520; ++j) {
             costmap_data[index++] = costmap->getCost(i, j);
         }
     }
 
     // Prepare tracking goal
     double tracking_goal[30] = {0.0};
     int track_idx = 0;
     for (const auto & p : transformed_plan.poses) {
         if (track_idx >= 30) break;
         tracking_goal[track_idx++] = p.pose.position.x;
         tracking_goal[track_idx++] = p.pose.position.y;
         tracking_goal[track_idx++] = tf2::getYaw(p.pose.orientation);
     }
 
     // Log inputs for debugging
     RCLCPP_DEBUG(logger_, "Current position: x=%f, y=%f, yaw=%f",
         current_position[0], current_position[1], current_position[2]);
     RCLCPP_DEBUG(logger_, "Current goal: x=%f, y=%f, yaw=%f",
         current_goal[0], current_goal[1], current_goal[2]);
     RCLCPP_DEBUG(logger_, "Tracking goal first point: x=%f, y=%f, yaw=%f",
         tracking_goal[0], tracking_goal[1], tracking_goal[2]);
     RCLCPP_DEBUG(logger_, "Costmap data sample: [%f, %f, %f, ...]",
         costmap_data[0], costmap_data[1], costmap_data[2]);
 
     // Solve NMPC
     double results[8] = {0.0};
     double trajectory[33] = {0.0};
     int status = nmpc_solver_->solve_my_mpc(
         current_position, costmap_data, current_goal, tracking_goal, results, trajectory);
 
     RCLCPP_INFO(logger_, "NMPC solver status: %d", status);
 
     if (status != ACADOS_SUCCESS) {
         RCLCPP_WARN(logger_, "NMPC solver failed with status %d", status);
         // Instead of throwing, return a zero velocity to avoid crashing
         geometry_msgs::msg::TwistStamped cmd_vel;
         cmd_vel.header.frame_id = pose.header.frame_id;
         cmd_vel.header.stamp = clock_->now();
         cmd_vel.twist.linear.x = 0.0;
         cmd_vel.twist.angular.z = 0.0;
         return cmd_vel;
     }
 
     // Extract control inputs
     double linear_vel = results[0];
     double angular_vel = results[1];
     RCLCPP_INFO(logger_, "Computed velocities: Linear vel=%f, Angular vel=%f", linear_vel, angular_vel);
 
     // Apply velocity constraints
     linear_vel = max(-desired_linear_vel_, min(linear_vel, desired_linear_vel_));
     angular_vel = max(-max_angular_vel_, min(angular_vel, max_angular_vel_));
     RCLCPP_INFO(logger_, "Constrained velocities: Linear vel=%f, Angular vel=%f", linear_vel, angular_vel);
 
     // Publish predicted trajectory for visualization
     nav_msgs::msg::Path predicted_path;
     predicted_path.header.frame_id = costmap_ros_->getBaseFrameID();
     predicted_path.header.stamp = clock_->now();
     for (int i = 0; i <= 10 && i * 3 < 33; ++i) {
         geometry_msgs::msg::PoseStamped traj_pose;
         traj_pose.header = predicted_path.header;
         traj_pose.pose.position.x = trajectory[i * 3];
         traj_pose.pose.position.y = trajectory[i * 3 + 1];
         traj_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), trajectory[i * 3 + 2]));
         predicted_path.poses.push_back(traj_pose);
     }
     static auto traj_pub = node_.lock()->create_publisher<nav_msgs::msg::Path>("predicted_trajectory", 1);
     traj_pub->publish(predicted_path);
 
     // Create velocity command
     geometry_msgs::msg::TwistStamped cmd_vel;
     cmd_vel.header.frame_id = pose.header.frame_id;
     cmd_vel.header.stamp = clock_->now();
     cmd_vel.twist.linear.x = linear_vel;
     cmd_vel.twist.angular.z = angular_vel;
 
     return cmd_vel;
 }
 
 void MPCController::setPlan(const nav_msgs::msg::Path & path)
 {
     global_plan_ = path;
     global_pub_->publish(path);
 }
 
 nav_msgs::msg::Path MPCController::transformGlobalPlan(
     const geometry_msgs::msg::PoseStamped & pose)
 {
     if (global_plan_.poses.empty()) {
         throw nav2_core::PlannerException("Received plan with zero length");
     }
 
     geometry_msgs::msg::PoseStamped robot_pose;
     if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)) {
         throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
     }
 
     nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
     double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                             costmap->getResolution() / 2.0;
 
     auto transformation_begin = min_by(
         global_plan_.poses.begin(), global_plan_.poses.end(),
         [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
             return euclidean_distance(robot_pose, ps);
         });
 
     auto transformation_end = std::find_if(
         transformation_begin, global_plan_.poses.end(),
         [&](const auto & global_plan_pose) {
             return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
         });
 
     auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
         geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
         stamped_pose.header.frame_id = global_plan_.header.frame_id;
         stamped_pose.header.stamp = pose.header.stamp;
         stamped_pose.pose = global_plan_pose.pose;
         transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
         return transformed_pose;
     };
 
     nav_msgs::msg::Path transformed_plan;
     std::transform(
         transformation_begin, transformation_end,
         std::back_inserter(transformed_plan.poses),
         transformGlobalPoseToLocal);
     transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
     transformed_plan.header.stamp = pose.header.stamp;
 
     global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);
     global_pub_->publish(transformed_plan);
 
     RCLCPP_INFO(logger_, "Transformed plan has %zu poses", transformed_plan.poses.size());
 
     if (transformed_plan.poses.empty()) {
         throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
     }
 
     return transformed_plan;
 }
 
 bool MPCController::transformPose(
     const std::shared_ptr<tf2_ros::Buffer> tf,
     const std::string frame,
     const geometry_msgs::msg::PoseStamped & in_pose,
     geometry_msgs::msg::PoseStamped & out_pose,
     const rclcpp::Duration & transform_tolerance)
 {
     if (in_pose.header.frame_id == frame) {
         out_pose = in_pose;
         return true;
     }
 
     try {
         // Convert rclcpp::Duration to tf2::Duration
         tf2::Duration tf_timeout = tf2::durationFromSec(transform_tolerance.seconds());
         tf->transform(in_pose, out_pose, frame, tf_timeout);
         return true;
     } catch (tf2::ExtrapolationException & ex) {
         try {
             auto transform = tf->lookupTransform(
                 frame, in_pose.header.frame_id, tf2::TimePointZero);
             tf2::Duration tf_timeout = tf2::durationFromSec(transform_tolerance.seconds());
             if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
                 RCLCPP_ERROR(
                     logger_,
                     "Transform data too old when converting from %s to %s",
                     in_pose.header.frame_id.c_str(), frame.c_str());
                 return false;
             } else {
                 tf2::doTransform(in_pose, out_pose, transform);
                 return true;
             }
         } catch (tf2::TransformException & ex2) {
             RCLCPP_ERROR(
                 logger_,
                 "Exception in transformPose (extrapolation): %s",
                 ex2.what());
             return false;
         }
     } catch (tf2::TransformException & ex) {
         RCLCPP_ERROR(
             logger_,
             "Exception in transformPose: %s",
             ex.what());
         return false;
     }
     return false;
 }
 
 }  // namespace mpc_low
 
 #include "pluginlib/class_list_macros.hpp"
 PLUGINLIB_EXPORT_CLASS(mpc_low::MPCController, nav2_core::Controller)