#include "mpc_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <stdexcept>
#include <cmath>

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
wheel_base_(0.5)
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
    declare_parameter_if_not_declared(node, plugin_name_ + ".blend_factor", rclcpp::ParameterValue(0.3)); // Reduced from 0.7
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

    nmpc_solver_ = std::make_unique<my_NMPC_solver>(nmpc_horizon_steps_);
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predicted_trajectory", 10);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
        "predicted_trajectory_markers", 10);
    goal_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
    publish_counter_ = 0;
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MPCController::odomCallback, this, std::placeholders::_1));
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

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    (void)velocity;
    (void)goal_checker;

    auto transformed_plan = transformGlobalPlan(pose);

    double current_position[3] = {
        pose.pose.position.x,
        pose.pose.position.y,
        normalize_angle(tf2::getYaw(pose.pose.orientation)) // Normalize robot yaw
    };


    double tracking_goal[30] = {0.0}; // Horizon of 10 steps (30 elements)

    RCLCPP_INFO(logger_, "Global plan size: %zu", transformed_plan.poses.size());
    if (transformed_plan.poses.size() > 1) {
        double x_diff = transformed_plan.poses[1].pose.position.x - transformed_plan.poses[0].pose.position.x;
        double y_diff = transformed_plan.poses[1].pose.position.y - transformed_plan.poses[0].pose.position.y;
        double spacing = std::sqrt(x_diff * x_diff + y_diff * y_diff);
        RCLCPP_INFO(logger_, "Global plan pose spacing: %.3f m", spacing);
    }
    double trajectory_length = 0.0;
    geometry_msgs::msg::PoseStamped selected_goal_pose;
    bool goal_found = false;
    double max_lookahead = 1.0;
    double min_lookahead = 1.0;
    static double last_x = current_position[0];  // Track progress
    static double last_y = current_position[1];
    double dist_moved = std::hypot(current_position[0] - last_x, current_position[1] - last_y);
    double dist_to_final = std::hypot(
        transformed_plan.poses.back().pose.position.x - current_position[0],
        transformed_plan.poses.back().pose.position.y - current_position[1]);
    for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
        double x_diff = transformed_plan.poses[i].pose.position.x - transformed_plan.poses[i-1].pose.position.x;
        double y_diff = transformed_plan.poses[i].pose.position.y - transformed_plan.poses[i-1].pose.position.y;
        trajectory_length += std::sqrt(x_diff * x_diff + y_diff * y_diff);

        // Use final goal if within 0.5 m, otherwise select goal 0.2–2.0 m ahead
        if (dist_to_final < 0.1 || trajectory_length >= min_lookahead && trajectory_length <= max_lookahead) {
            selected_goal_pose = (dist_to_final < 0.1) ? transformed_plan.poses.back() : transformed_plan.poses[i];
            goal_found = true;
            break;
        }
    }
    if (!goal_found && !transformed_plan.poses.empty()) {
        selected_goal_pose = transformed_plan.poses.back();
        goal_found = true;
        RCLCPP_INFO(logger_, "Using final path point as goal.");
    }else if (dist_moved < 0.01 && dist_to_final > 1.0) {  // Stuck detection
        selected_goal_pose = transformed_plan.poses.back();
        goal_found = true;
        RCLCPP_INFO(logger_, "Stuck detected (moved %.3f m), selecting final goal.", dist_moved);
        
    }
    // Update last position
last_x = current_position[0];
last_y = current_position[1];

    double current_robot_goal[3] = {0.0};
    if (goal_found) {
        current_robot_goal[0] = selected_goal_pose.pose.position.x - current_position[0];
        current_robot_goal[1] = selected_goal_pose.pose.position.y - current_position[1];
        double angle_goal = 2* std::atan2( selected_goal_pose.pose.orientation.z, selected_goal_pose.pose.orientation.w);
        current_robot_goal[2] = angle_goal;

        double dist_to_final = std::sqrt(
            (transformed_plan.poses.back().pose.position.x - current_position[0])*transformed_plan.poses.back().pose.position.x - current_position[0])
            + (transformed_plan.poses.back().pose.position.y - current_position[1])*(transformed_plan.poses.back().pose.position.y - current_position[1]);

    } else {
        RCLCPP_WARN(logger_, "No valid goal found in transformed plan");
    }
    RCLCPP_INFO(logger_, "Selected goal: [%.3f, %.3f, %.3f], Distance to final goal: %.3f",
        current_robot_goal[0], current_robot_goal[1], current_robot_goal[2], dist_to_final);

        
    double costmap_data[520] = {0.0}; // 260*2

    double results[8] = {0.0};
    double trajectory[33] = {0.0};
    double weights[6] = {weight_x_, weight_y_, weight_yaw_, weight_terminal_x_, weight_terminal_y_, weight_terminal_};
    double fake_position[3] = {0.0};
    fake_position[3] = current_position[3];

    
    //double fake_goal[3] = {1.0, 0.5, 0.0}; 

    geometry_msgs::msg::PoseStamped fake_goal_pose;
    fake_goal_pose.header.frame_id = "map";
    fake_goal_pose.pose.position.x = current_robot_goal[0];
    fake_goal_pose.pose.position.y = current_robot_goal[1];
    fake_goal_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_robot_goal[2]);
    fake_goal_pose.pose.orientation = tf2::toMsg(q);

    publishGoalMarker(selected_goal_pose);


    int status = nmpc_solver_->solve_my_mpc(fake_position, costmap_data, current_robot_goal, tracking_goal, results, trajectory, weights);
    double phi_dot_1 = results[0];  // left wheel velocity (rad/s)
    double phi_dot_2 = results[1];  // right wheel velocity (rad/s)
    double linear_vel = (r_wheel_ / (2.0)) * (phi_dot_1 + phi_dot_2);
    double angular_vel = (r_wheel_ / (alpha_ * B_)) * (phi_dot_1 - phi_dot_2);

    RCLCPP_INFO(logger_, "Computed controls: phi_dot_1=%.3f, phi_dot_2=%.3f, linear=%.3f, angular=%.3f",
        phi_dot_1, phi_dot_2, linear_vel, angular_vel);

    if (status != ACADOS_SUCCESS) {
        RCLCPP_WARN(logger_, "NMPC solver failed with status %d, resetting solver", status);
        nmpc_solver_->reset_solver();
        nmpc_solver_ = std::make_unique<my_NMPC_solver>(nmpc_horizon_steps_);
        linear_vel = 0.0;
        double yaw_diff = std::atan2(current_robot_goal[1], current_robot_goal[0]) - current_position[2];
        yaw_diff = normalize_angle(yaw_diff); // Normalize fallback yaw difference
        angular_vel = std::max(-0.5, std::min(0.5, yaw_diff * 1.0));
    } 

    const double smoothing_factor = 0.1;
    linear_vel = smoothing_factor * prev_linear_vel_ + (1.0 - smoothing_factor) * linear_vel;
    angular_vel = smoothing_factor * prev_angular_vel_ + (1.0 - smoothing_factor) * angular_vel;
    prev_linear_vel_ = linear_vel;
    prev_angular_vel_ = angular_vel;

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
    // marker.header.frame_id = "map";
    // marker.header.stamp = clock_->now();
    marker.header = goal_pose.header;  // Use the same header as the goal pose
    marker.ns = "local_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = goal_pose.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    if (!goal_marker_pub_->is_activated()) {
        RCLCPP_WARN(logger_, "Goal marker publisher not active!");
        return;
    }
    
    goal_marker_pub_->publish(marker);
}

void MPCController::publishFinalGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_plan_.header.frame_id;  // Use original frame
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