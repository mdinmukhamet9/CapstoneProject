#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <iostream>
#include <fstream>
#include <array>
#include <cmath>
#include <memory>

class GoalFollower : public rclcpp::Node
{
public:
    GoalFollower() 
        : Node("goal_follower")
    {
        // Declare parameters
        this->declare_parameter("goal_topic", "/j100_0001/received_global_plan");
        this->declare_parameter("costmap_topic", "/j100_0001/local_costmap/costmap");
        this->declare_parameter("amcl_pose_topic", "/j100_0001/amcl_pose");
        this->declare_parameter("data_file_path", "data_low.csv");
        this->declare_parameter("perf_file_path", "data_perf.csv");

        // Get parameter values
        goal_topic_ = this->get_parameter("goal_topic").as_string();
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        amcl_pose_topic_ = this->get_parameter("amcl_pose_topic").as_string();
        std::string data_file_path = this->get_parameter("data_file_path").as_string();
        std::string perf_file_path = this->get_parameter("perf_file_path").as_string();

        // Open files
        myfile_.open(data_file_path, std::ios::out);
        if (!myfile_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open data file: %s", data_file_path.c_str());
        }
        myperffile_.open(perf_file_path, std::ios::out);
        if (!myperffile_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open performance file: %s", perf_file_path.c_str());
        }

        // Initialize subscribers
        goal_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            goal_topic_, 1,
            std::bind(&GoalFollower::goal_callback, this, std::placeholders::_1));

        costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 1,
            std::bind(&GoalFollower::costmap_callback, this, std::placeholders::_1));

        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_pose_topic_, 1,
            std::bind(&GoalFollower::amcl_pose_callback, this, std::placeholders::_1));

        // Initialize publishers
        goal_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_arrow", 1);
        robot_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_arrow", 1);

        RCLCPP_INFO(this->get_logger(), "GoalFollower Node Initialized");
    }

    ~GoalFollower() {
        if (myfile_.is_open()) myfile_.close();
        if (myperffile_.is_open()) myperffile_.close();
    }

private:
    void goal_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        double trajectory_length = 0.0;
        size_t goal_index = 0;
        for (size_t i = 1; i < msg->poses.size(); ++i) {
            double x_diff = msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x;
            double y_diff = msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y;
            trajectory_length += std::sqrt(x_diff * x_diff + y_diff * y_diff);
            goal_index = i;
            if (trajectory_length >= 1.0) break;
        }

        if (goal_index > 0) {
            goal_status_ = 1;
            goal_pose_[0] = msg->poses[goal_index].pose.position.x;
            goal_pose_[1] = msg->poses[goal_index].pose.position.y;
            goal_pose_[2] = std::atan2(
                msg->poses[goal_index].pose.position.y - msg->poses[goal_index - 1].pose.position.y,
                msg->poses[goal_index].pose.position.x - msg->poses[goal_index - 1].pose.position.x);
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.frame_id = "map";
            goal_msg.header.stamp = this->get_clock()->now();
            goal_msg.pose.position.x = goal_pose_[0];
            goal_msg.pose.position.y = goal_pose_[1];
            goal_msg.pose.position.z = 0.0;
    
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pose_[2]);  // Convert yaw to quaternion
            goal_msg.pose.orientation = tf2::toMsg(q);
    
            goal_arrow_pub_->publish(goal_msg);
            RCLCPP_INFO(this->get_logger(), "Published goal arrow at (%.2f, %.2f) with yaw %.2f", 
                        goal_pose_[0], goal_pose_[1], goal_pose_[2]);
        } else {
            goal_status_ = 0;
        }



        RCLCPP_INFO(this->get_logger(), "Trajectory Received: Size %ld, Length %f", msg->poses.size(), trajectory_length);
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_pose_[0] = msg->pose.pose.position.x;
        current_pose_[1] = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_[2] = yaw;
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        int obst_count = 0;
        double resolution = msg->info.resolution;

        for (size_t i = 0; i < msg->data.size(); ++i) {
            if (msg->data[i] > 99) {
                costmap_data_[2 * obst_count] = (i % msg->info.width) * resolution + msg->info.origin.position.x;
                costmap_data_[2 * obst_count + 1] = (i / msg->info.width) * resolution + msg->info.origin.position.y;
                if (++obst_count == costmap_data_.size() / 2) break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Obstacles Count: %d", obst_count);
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_arrow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_arrow_pub_;

    // Internal Variables
    std::array<double, 6> goal_pose_ = {0.0};
    std::array<double, 6> current_pose_ = {0.0};
    std::array<double, 520> costmap_data_ = {-100.0};
    std::array<double, 30> tracking_goal_ = {0.0};
    int goal_status_ = 0;

    // File streams
    std::ofstream myfile_;
    std::ofstream myperffile_;

    // Topic names
    std::string goal_topic_;
    std::string costmap_topic_;
    std::string amcl_pose_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}