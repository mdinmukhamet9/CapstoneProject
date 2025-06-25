#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "acados_solver_holder.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "yolo_msgs/msg/detection_array.hpp"
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
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    geometry_msgs::msg::PoseStamped selectGoalPose(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const nav_msgs::msg::Path& global_plan,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose);
    void publishFinalGoalMarker(const geometry_msgs::msg::PoseStamped& goal_pose);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg); // New callback

protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
    void processCostmap(const nav2_costmap_2d::Costmap2D* costmap, double* costmap_data, const double current_position[3], int max_obstacles = 260);

    bool transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
    rclcpp::Clock::SharedPtr clock_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav_msgs::msg::Path global_plan_;
    rclcpp::Duration transform_tolerance_;
    double desired_linear_vel_;
    double max_angular_vel_;
    int nmpc_horizon_steps_;

    std::unique_ptr<my_NMPC_solver> nmpc_solver_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // Changed to Marker

    // Added for velocity smoothing
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Time last_marker_publish_time_;
    int marker_id_counter_;
    double marker_publish_interval_;
    // Add after existing members
    int max_obstacles_;
    double weight_x_;
    double weight_y_; 
    double weight_yaw_;
    double weight_terminal_x_;
    double weight_terminal_y_;
    double weight_terminal_;
    double blend_factor_;
    double prev_linear_vel_;
    double prev_angular_vel_;
    int publish_counter_;  // Added missing member
    double wheel_base_;  // Added member variable for wheel base
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    double r_wheel_;  // Wheel radius
    double alpha_;    // Scaling factor for angular velocity
    double B_;  // Distance between wheels
    double tracking_goal_[30];  // Added member variable for tracking_goal
    double min_lookahead_distance_ = 0.8;
    double max_lookahead_distance_ = 1.5;
    double stuck_threshold_ = 0.02;
    double stuck_lookahead_ = 0.7;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double actual_linear_vel_;  // New member for actual linear velocity
    double actual_angular_vel_;
    unsigned int stuck_counter_ = 0; 
    std::ofstream goal_data_csv_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // SSM-related members
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
    geometry_msgs::msg::Point person_position_;  // Store person position in base_link frame
    bool person_detected_ = false;  // Flag for person detection

    void detectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
};

}  // namespace mpc_low

#endif  // MPC_CONTROLLER_HPP_