#include "mpc_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <stdexcept>
#include <cmath>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using nav2_util::geometry_utils::min_by;

namespace mpc_low
{

// Add normalize_angle function
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

MPCController::MPCController(): transform_tolerance_(rclcpp::Duration::from_seconds(0.0)),
desired_linear_vel_(0.5),
max_angular_vel_(1.0),
nmpc_horizon_steps_(5),
prev_linear_vel_(0.0),
prev_angular_vel_(0.0),
publish_counter_(0),
wheel_base_(0.5),
actual_linear_vel_(0.0),
actual_angular_vel_(0.0)
{std::fill(tracking_goal_, tracking_goal_ + 30, 0.0);}

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
    tf_buffer_ = tf;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".nmpc_horizon_steps", rclcpp::ParameterValue(10));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".wheel_base", rclcpp::ParameterValue(0.3));
    
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_obstacles", rclcpp::ParameterValue(260));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_x", rclcpp::ParameterValue(100.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_y", rclcpp::ParameterValue(100.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_yaw", rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_terminal_x", rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_terminal_y", rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".weight_terminal", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".blend_factor", rclcpp::ParameterValue(0.3)); 
    declare_parameter_if_not_declared(node, plugin_name_ + ".r_wheel", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".alpha", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".B", rclcpp::ParameterValue(0.3765));
    node->get_parameter(plugin_name_ + ".r_wheel", r_wheel_);
    node->get_parameter(plugin_name_ + ".alpha", alpha_);
    node->get_parameter(plugin_name_ + ".B", B_);
    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".nmpc_horizon_steps", nmpc_horizon_steps_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    node->get_parameter(plugin_name_ + ".wheel_base", wheel_base_);

    node->get_parameter(plugin_name_ + ".max_obstacles", max_obstacles_);
    node->get_parameter(plugin_name_ + ".weight_x", weight_x_);
    node->get_parameter(plugin_name_ + ".weight_y", weight_y_);
    node->get_parameter(plugin_name_ + ".weight_yaw", weight_yaw_);
    node->get_parameter(plugin_name_ + ".weight_terminal", weight_terminal_);
    node->get_parameter(plugin_name_ + ".weight_terminal_x", weight_terminal_x_);
    node->get_parameter(plugin_name_ + ".weight_terminal_y", weight_terminal_y_);
    node->get_parameter(plugin_name_ + ".blend_factor", blend_factor_);
    node->declare_parameter("min_lookahead_distance", 0.5);
    node->declare_parameter("max_lookahead_distance", 1.5);
    node->get_parameter("min_lookahead_distance", min_lookahead_distance_);
    node->get_parameter("max_lookahead_distance", max_lookahead_distance_);

    nmpc_solver_ = std::make_unique<my_NMPC_solver>(nmpc_horizon_steps_);
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predicted_trajectory", 10);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
        "predicted_trajectory_markers", 10);
    goal_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
    publish_counter_ = 0;
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MPCController::odomCallback, this, std::placeholders::_1));
    detection_sub_ = node->create_subscription<yolo_msgs::msg::DetectionArray>(
        "/yolo/detections_3d", 10, std::bind(&MPCController::detectionCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/j100_0001/cmd_vel", 10, std::bind(&MPCController::cmdVelCallback, this, std::placeholders::_1)); // New subscription    
    
    
    goal_data_csv_.open("mpc_goal_data.csv");
    goal_data_csv_ << "time_stamp,robot_x,robot_y,robot_yaw,selected_goal_x,selected_goal_y,final_goal_x,final_goal_y,linear_vel,angular_vel,actual_linear_vel,actual_angular_vel\n";
}

void MPCController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    actual_linear_vel_ = msg->linear.x;
    actual_angular_vel_ = msg->angular.z;
    // RCLCPP_DEBUG(logger_, "Received cmd_vel: linear.x=%.3f, angular.z=%.3f", actual_linear_vel_, actual_angular_vel_);
}

void MPCController::detectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
    person_detected_ = false;
    for (const auto& detection : msg->detections) {
        if (detection.class_name == "person") {
            person_position_ = detection.bbox3d.center.position;
            // Transform from camera_0_link to base_link (camera offset: 0.1, 0.0, 0.18)
            person_position_.x -= 0.1;  
            person_position_.z -= 0.18; 
            person_detected_ = true;
            RCLCPP_INFO(logger_,
                        "Person detected in base_link at (x=%.2f, y=%.2f, z=%.2f)",
                        person_position_.x, person_position_.y, person_position_.z);
            break;
        }
    }
}

void MPCController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(logger_, "Odometry Position: [%.3f, %.3f, %.3f], Velocity: [%.3f, %.3f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                tf2::getYaw(msg->pose.pose.orientation),
                msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}
void MPCController::cleanup()
{
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type mpc_low::MPCController",
        plugin_name_.c_str());
    nmpc_solver_.reset();
    global_pub_.reset();
    traj_pub_.reset();
    marker_pub_.reset();
    cmd_vel_sub_.reset();
    detection_sub_.reset();
    
    if (goal_data_csv_.is_open()) {
        goal_data_csv_.close();
    }
}

void MPCController::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type mpc_low::MPCController",
        plugin_name_.c_str());
    global_pub_->on_activate();
    traj_pub_->on_activate();
    marker_pub_->on_activate();
    goal_marker_pub_->on_activate();
}

void MPCController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type mpc_low::MPCController",
        plugin_name_.c_str());
    global_pub_->on_deactivate();
    traj_pub_->on_deactivate();
    marker_pub_->on_deactivate();
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
}

geometry_msgs::msg::PoseStamped MPCController::selectGoalPose(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& global_plan,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
{
    geometry_msgs::msg::PoseStamped goal;
    goal.header = global_plan.header;
    if (global_plan.poses.empty()) {
        RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "Empty path received");
        return goal;
    }
    const double current_x = current_pose.pose.position.x;
    const double current_y = current_pose.pose.position.y;
    const auto& final_pose = global_plan.poses.back().pose.position;
    const double dist_to_final = std::hypot(
        final_pose.x - current_x,
        final_pose.y - current_y);
    double lookahead = min_lookahead_distance_;
    if (dist_to_final > max_lookahead_distance_) {
        lookahead = max_lookahead_distance_;
    } else if (dist_to_final > min_lookahead_distance_) {
        lookahead = dist_to_final;
    }
    size_t goal_index = global_plan.poses.size() - 1;
    double accumulated_dist = 0.0;
    for (size_t i = 1; i < global_plan.poses.size(); ++i) {
        const double dx = global_plan.poses[i].pose.position.x - 
                        global_plan.poses[i-1].pose.position.x;
        const double dy = global_plan.poses[i].pose.position.y - 
                        global_plan.poses[i-1].pose.position.y;
        accumulated_dist += std::hypot(dx, dy);

        if (accumulated_dist >= lookahead) {
            goal_index = i;
            break;
        }
    }
    double actual_dist = std::hypot(
        global_plan.poses[goal_index].pose.position.x - current_x,
        global_plan.poses[goal_index].pose.position.y - current_y);
    if (actual_dist < min_lookahead_distance_ && goal_index < global_plan.poses.size() - 1) {
        for (size_t i = goal_index + 1; i < global_plan.poses.size(); ++i) {
            actual_dist = std::hypot(
                global_plan.poses[i].pose.position.x - current_x,
                global_plan.poses[i].pose.position.y - current_y);
            
            if (actual_dist >= min_lookahead_distance_ || i == global_plan.poses.size() - 1) {
                goal_index = i;
                break;
            }
        }
    }

    goal = global_plan.poses[goal_index];
    if (goal_index < global_plan.poses.size() - 1) {
        const auto& next_pose = global_plan.poses[goal_index + 1];
        const double yaw = std::atan2(
            next_pose.pose.position.y - goal.pose.position.y,
            next_pose.pose.position.x - goal.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation = tf2::toMsg(q);
    } else if (goal_index > 0) {
        const auto& prev_pose = global_plan.poses[goal_index - 1];
        const double yaw = std::atan2(
            goal.pose.position.y - prev_pose.pose.position.y,
            goal.pose.position.x - prev_pose.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation = tf2::toMsg(q);
    }
    return goal;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    (void)velocity;
    (void)goal_checker;

    auto transformed_plan = transformGlobalPlan(pose);
    RCLCPP_INFO(logger_, "Global plan size: %zu", transformed_plan.poses.size());

    // Get current position
    double current_position[3] = {
        pose.pose.position.x,
        pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation)
    };

    // Select goal pose using the unified function
    geometry_msgs::msg::PoseStamped selected_goal_pose = selectGoalPose(pose, transformed_plan, logger_, clock_);
    publishGoalMarker(selected_goal_pose);

    // Calculate relative goal (FIXED SECTION)
    double current_robot_goal[3] = {0.0};
    bool goal_found = !transformed_plan.poses.empty();
    
    if (goal_found) {
        // --- NEW CODE: Transform goal into robot's frame ---
        try {
            geometry_msgs::msg::PoseStamped goal_in_robot_frame;
            tf_buffer_->transform(selected_goal_pose, goal_in_robot_frame, pose.header.frame_id, tf2::durationFromSec(0.1));
            
            current_robot_goal[0] = goal_in_robot_frame.pose.position.x;
            current_robot_goal[1] = goal_in_robot_frame.pose.position.y;
            current_robot_goal[2] = tf2::getYaw(goal_in_robot_frame.pose.orientation);

            // RCLCPP_DEBUG(logger_, "Goal in Robot Frame: (%.3f, %.3f, %.3f)",
            //     current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger_, "TF Exception: %s", ex.what());
            // Fallback to simple difference (less accurate)
            current_robot_goal[0] = selected_goal_pose.pose.position.x - current_position[0];
            current_robot_goal[1] = selected_goal_pose.pose.position.y - current_position[1];
            current_robot_goal[2] = tf2::getYaw(selected_goal_pose.pose.orientation) - current_position[2];
        }

    } else {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,  
            "No valid goal found in transformed plan (size: %zu)", 
            transformed_plan.poses.size());
    }
    
    // Extract costmap as a grid (not obstacle positions)
    double costmap_data[520] = {0.0}; // 260*2, 260 nearest points aproximately 10% of all costmap
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    double resolution = costmap->getResolution();
    unsigned int width = costmap->getSizeInCellsX();
    unsigned int height = costmap->getSizeInCellsY();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();

    int obstacle_count = 0;
    for (unsigned int y = 0; y < height && obstacle_count < max_obstacles_; ++y) {
        for (unsigned int x = 0; x < width && obstacle_count < max_obstacles_; ++x) {
            unsigned char cost = costmap->getCost(x, y);
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) {
                double world_x, world_y;
                costmap->mapToWorld(x, y, world_x, world_y);
                // Transform to base_link frame
                geometry_msgs::msg::PoseStamped world_pose, base_pose;
                world_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                world_pose.header.stamp = pose.header.stamp;
                world_pose.pose.position.x = world_x;
                world_pose.pose.position.y = world_y;
                world_pose.pose.position.z = 0.0;
                world_pose.pose.orientation.w = 1.0;
                try {
                    transformPose(tf_, costmap_ros_->getBaseFrameID(), world_pose, base_pose, transform_tolerance_);
                    costmap_data[obstacle_count * 2] = base_pose.pose.position.x;
                    costmap_data[obstacle_count * 2 + 1] = base_pose.pose.position.y;
                    obstacle_count++;
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(logger_, "Failed to transform obstacle position: %s", ex.what());
                }
            }
        }
    }
    // RCLCPP_INFO(logger_, "Found %d obstacles in costmap", obstacle_count);



    double results[8] = {0.0};
    double trajectory[33] = {0.0};
    double weights[6] = {weight_x_, weight_y_, weight_yaw_, weight_terminal_x_, weight_terminal_y_, weight_terminal_};
    double person_pos[3] = {0.0};
    if (person_detected_) {
        person_pos[0] = person_position_.x;
        person_pos[1] = person_position_.y;
        person_pos[2] = person_position_.z;
    }

    
    // RCLCPP_INFO(logger_, "CURRENT ROBOT POSITION: %.3f, %.3f, %.3f, CURR ROBOT GOAL: %.3f, %.3f, %.3f \n",
    //     current_position[0], current_position[1], current_position[2], current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);

    int status = nmpc_solver_->solve_my_mpc(current_position, costmap_data, current_robot_goal, tracking_goal_, results, trajectory, weights, person_detected_, person_pos);
    double phi_dot_1 = results[0];  // left wheel velocity (rad/s)
    double phi_dot_2 = results[1];  // right wheel velocity (rad/s)
    double linear_vel = (r_wheel_ / (2.0)) * (phi_dot_1 + phi_dot_2);
    double angular_vel = (r_wheel_ / (alpha_ * B_)) * (phi_dot_1 - phi_dot_2);

    if (person_detected_) {
        double dx = person_position_.x;
        double dy = person_position_.y;
        double distance = sqrt(dx * dx + dy * dy);
        const double r_robot = 0.254;  
        const double r_human = 0.3;  
        double min_safe_distance = r_robot + r_human;
        if (distance < min_safe_distance) {
            // Stop the robot if too close
            linear_vel = 0.0;
            angular_vel = 0.0;
            phi_dot_1 = 0.0;
            phi_dot_2 = 0.0;
            RCLCPP_WARN(logger_, "Human too close (%.3f < %.3f), stopping robot", distance, min_safe_distance);
        } else {
            double max_v_squared = distance * distance - min_safe_distance * min_safe_distance;
            if (max_v_squared < 0) max_v_squared = 0.0;
            double current_v_squared = linear_vel * linear_vel;
            if (current_v_squared > max_v_squared) {
                double scale = sqrt(max_v_squared / current_v_squared);
                phi_dot_1 *= scale;
                phi_dot_2 *= scale;
                linear_vel *= scale;
                angular_vel *= scale;
                RCLCPP_INFO(logger_, "SSM adjusted velocity: scale=%.3f, new linear_vel=%.3f", scale, linear_vel);
            }
        }
    }



    double time_er = clock_ ->now().seconds();
    RCLCPP_INFO(logger_, "Computed controls: phi_dot_1=%.3f, phi_dot_2=%.3f, linear=%.3f, angular=%.3f \n",
        phi_dot_1, phi_dot_2, linear_vel, angular_vel);

    if (status != ACADOS_SUCCESS) {
        RCLCPP_WARN(logger_, "NMPC solver failed with status %d, resetting solver \n", status);
        nmpc_solver_->reset_solver();
        nmpc_solver_ = std::make_unique<my_NMPC_solver>(nmpc_horizon_steps_);
        linear_vel = 0.0;
        double yaw_diff = std::atan2(current_robot_goal[1], current_robot_goal[0]) - current_position[2];
        yaw_diff = normalize_angle(yaw_diff); // Normalize fallback yaw difference
        angular_vel = std::max(-0.5, std::min(0.5, yaw_diff * 1.0));
    } 



    if (goal_data_csv_.is_open() && goal_found) {
        goal_data_csv_ 
                    << time_er <<","
                    << current_position[0] << ","
                    << current_position[1] << ","
                    << current_position[2] << ","
                    << selected_goal_pose.pose.position.x << ","
                    << selected_goal_pose.pose.position.y << ","
                    << transformed_plan.poses.back().pose.position.x << ","
                    << transformed_plan.poses.back().pose.position.y << ","
                    << linear_vel << ","
                    << angular_vel << ","
                    << actual_linear_vel_ << "," // New fields
                    << actual_angular_vel_ << "\n";
        goal_data_csv_.flush(); // Ensure data is written immediately
    }
    publish_counter_++;
    if (publish_counter_ % 1 == 0) {
        nav_msgs::msg::Path predicted_path;
        predicted_path.header.frame_id = "base_link";
        predicted_path.header.stamp = clock_->now();

        for (int i = 0; i < 11; ++i) {
            geometry_msgs::msg::PoseStamped traj_pose;
            traj_pose.header = predicted_path.header;
            traj_pose.pose.position.x = trajectory[i * 3];
            traj_pose.pose.position.y = trajectory[i * 3 + 1];
            traj_pose.pose.position.z = 0.0;
            double yaw = trajectory[i * 3 + 2] + current_position[2];
            yaw = normalize_angle(yaw); // Normalize trajectory yaw for visualization
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            traj_pose.pose.orientation = tf2::toMsg(q);
            predicted_path.poses.push_back(traj_pose);
        }

        if (traj_pub_->is_activated()) {
            traj_pub_->publish(predicted_path);
        }


        publish_counter_ = 0;
    }


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
    std::fill(tracking_goal_, tracking_goal_ + 30, 0.0);

    global_pub_->publish(path);

    if (!global_plan_.poses.empty()) {
        publishFinalGoalMarker(global_plan_.poses.back());
    }
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
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    while (transformation_begin != global_plan_.poses.end()) {
        double dx = transformation_begin->pose.position.x - robot_pose.pose.position.x;
        double dy = transformation_begin->pose.position.y - robot_pose.pose.position.y;
        double angle_to_point = std::atan2(dy, dx);
        if (std::abs(normalize_angle(angle_to_point - robot_yaw)) < M_PI/2) {
            break;  // Point is in front of robot
        }
        ++transformation_begin;
    }

    auto transformation_end = std::find_if(
        transformation_begin, global_plan_.poses.end(),
        [&, dist_threshold](const auto & global_plan_pose) {
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
void MPCController::publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = goal_pose.header.frame_id;
    marker.header.stamp = clock_->now();
    marker.ns = "local_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = goal_pose.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    if (!goal_marker_pub_->is_activated()) {
        RCLCPP_WARN(logger_, "Goal marker publisher not active!");
        return;
    }
    
    goal_marker_pub_->publish(marker);
}

void MPCController::publishFinalGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_plan_.header.frame_id;
    marker.header.stamp = clock_->now();
    marker.ns = "final_goal";
    marker.id = 1;  // Different ID from the local goal marker
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position and orientation
    marker.pose = goal_pose.pose;
    
    // Set marker scale (in meters)
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // Bright red color for final goal
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    // Persistent marker (doesn't disappear)
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means infinite
    
    if (!goal_marker_pub_->is_activated()) {
        goal_marker_pub_->on_activate();
    }
    
    goal_marker_pub_->publish(marker);
    RCLCPP_INFO(logger_, "Published FINAL goal marker at [%.2f, %.2f]", 
        goal_pose.pose.position.x, goal_pose.pose.position.y);
}


}  // namespace mpc_low

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_low::MPCController, nav2_core::Controller)