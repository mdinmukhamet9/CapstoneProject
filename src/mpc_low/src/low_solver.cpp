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
#include <vector>
#include <chrono>
#include <mutex>

#include "acados_solver_holder.hpp"

class GoalFollower : public rclcpp::Node
{
public:
    GoalFollower() 
        : Node("goal_follower"), myMpcSolver(5)
    {
        initialize_parameters();
        initialize_publishers();
        initialize_subscribers();
        initialize_timer();
        open_files();
        
        RCLCPP_INFO(this->get_logger(), "GoalFollower Node Initialized");
    }

    ~GoalFollower() {
        if (!myfile_.is_open()) myfile_.open(data_file_path_, std::ios::out);
        if (myfile_.is_open()) myfile_.close();
        if (myperffile_.is_open()) myperffile_.close();
    }

private:
    
    double gazebo_time_ = 0.0;
    std::array<double, 7> goal_from_target_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    void initialize_parameters() {
        // Declare and get parameters
        // Declare and get parameters with updated topic names
        this->declare_parameter("goal_topic", "/j100_0001/received_global_plan");  // Updated from /received_global_plan
        this->declare_parameter("costmap_topic", "/j100_0001/local_costmap/costmap");  // Updated from /local_costmap/costmap
        this->declare_parameter("amcl_pose_topic", "/j100_0001/amcl_pose");  // Updated from /amcl_pose
        this->declare_parameter("data_file_path", "data_low.csv");
        this->declare_parameter("perf_file_path", "data_perf.csv");
        this->declare_parameter("control_rate", 40.0);
        this->declare_parameter("mpc_horizon", 5);

        goal_topic_ = this->get_parameter("goal_topic").as_string();
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        amcl_pose_topic_ = this->get_parameter("amcl_pose_topic").as_string();
        data_file_path_ = this->get_parameter("data_file_path").as_string();
        perf_file_path_ = this->get_parameter("perf_file_path").as_string();
        control_rate_ = this->get_parameter("control_rate").as_double();
        mpc_horizon_ = this->get_parameter("mpc_horizon").as_int();

    }

    void initialize_publishers() {
        goal_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_arrow", 1);
        robot_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_arrow", 1);
        nmpc_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("nmpc_path", 10);
        motor_controller_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/j100_0001/my_robot_velocity", 1); // /joint_group_velocity_controller/command
    }

    void initialize_subscribers() {
        goal_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            goal_topic_, 1,
            [this](const nav_msgs::msg::Path::ConstSharedPtr &msg) { goal_callback(msg); });

        costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 1,
            [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) { costmap_callback(msg); });

        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_pose_topic_, 1,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) { amcl_pose_callback(msg); });

        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,  
            [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr &msg) { this->clock_callback(msg); });
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::ConstSharedPtr &msg) {
        std::lock_guard<std::mutex> lock(clock_mutex_);  
        rclcpp::Time ros_time(msg->clock);
        gazebo_time_ = ros_time.seconds();
        RCLCPP_DEBUG(this->get_logger(), "Received clock time: %.3f", gazebo_time_);
    }


    void initialize_timer() {
        auto timer_period = std::chrono::duration<double>(1.0/control_rate_);
        control_timer_ = this->create_wall_timer(
            timer_period,
            [this]() { control_loop(); });
    }

    void open_files() {
        myfile_.open(data_file_path_, std::ios::out);
        myperffile_.open(perf_file_path_, std::ios::out);
        if (!myfile_.is_open() || !myperffile_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open data files!");
        }
    }

    void goal_callback(const nav_msgs::msg::Path::ConstSharedPtr &msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            goal_status_ = 0;
            return;
        }

        double trajectory_length = 0.0;
        size_t goal_index = 0;
        for (size_t i = 1; i < msg->poses.size(); ++i) {
            const auto& prev = msg->poses[i-1].pose.position;
            const auto& curr = msg->poses[i].pose.position;
            trajectory_length += std::hypot(curr.x - prev.x, curr.y - prev.y);
            goal_index = i;
            if (trajectory_length >= 2.0) break;
        }

        if (goal_index > 0) {
            
            goal_status_ = 1;
            const auto& goal_pose = msg->poses[goal_index].pose;
            goal_pose_[0] = goal_pose.position.x;
            goal_pose_[1] = goal_pose.position.y;
            
            // Calculate orientation from path segment
            const auto& prev_pose = msg->poses[goal_index-1].pose.position;
            goal_pose_[2] = std::atan2(
                goal_pose.position.y - prev_pose.y,
                goal_pose.position.x - prev_pose.x);

            // Populate goal_from_target_ for visualization
            goal_from_target_[0] = goal_pose.position.x;
            goal_from_target_[1] = goal_pose.position.y;
            goal_from_target_[2] = goal_pose.position.z;
            goal_from_target_[3] = goal_pose.orientation.x;
            goal_from_target_[4] = goal_pose.orientation.y;
            goal_from_target_[5] = goal_pose.orientation.z;
            goal_from_target_[6] = goal_pose.orientation.w;

            publish_goal_arrow();
        }

        RCLCPP_INFO(this->get_logger(), "Received trajectory: %zu poses, length: %.2fm", 
                   msg->poses.size(), trajectory_length);
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
        current_pose_[0] = msg->pose.pose.position.x;
        current_pose_[1] = msg->pose.pose.position.y;
        
        // Calculate yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_pose_[2] = 2.0 * std::atan2(q.z(), q.w());

    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
        costmap_info_ = msg->info;
        costmap_data_.fill(-100.0);

        size_t obst_count = 0;
        for (size_t i = 0; i < msg->data.size(); ++i) {
            if (msg->data[i] > 99) {
                size_t x = i % msg->info.width;
                size_t y = i / msg->info.width;
                costmap_data_[2*obst_count] = x * msg->info.resolution + msg->info.origin.position.x;
                costmap_data_[2*obst_count+1] = y * msg->info.resolution + msg->info.origin.position.y;
                if (++obst_count >= costmap_data_.size()/2) break;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Processed %zu obstacles", obst_count);
    }

    void control_loop() {
        const double current_error = calculate_current_error();
        if (!(goal_status_ || current_error > 0.10)) {
            return;
        }

        process_mpc_solution();
        publish_motor_commands();
        //publish_trajectory();
        publish_robot_arrow();
        log_data();
    }

    double calculate_current_error() const {
        const double dx = goal_pose_[0] - current_pose_[0];
        const double dy = goal_pose_[1] - current_pose_[1];
        const double dtheta = goal_pose_[2] - current_pose_[2];
        return dx*dx + dy*dy + dtheta*dtheta;
    }

    void process_mpc_solution() {
        std::array<double, 520> transformed_costmap = transform_costmap();
        std::array<double, 30> tracking_goal = calculate_tracking_goal();
        std::array<double, 3> current_robot_position = {0.0, 0.0, current_pose_[2]};
        std::array<double, 3> current_robot_goal = {
            goal_pose_[0] - current_pose_[0],
            goal_pose_[1] - current_pose_[1],
            goal_pose_[2]
        };

        double result[8] = {0};
        double trajectory[33] = {0};
        const int status = myMpcSolver.solve_my_mpc(
            current_robot_position.data(),
            transformed_costmap.data(),
            current_robot_goal.data(),
            tracking_goal.data(),
            result,
            trajectory
        );
        update_trajectory_data(trajectory);
        RCLCPP_INFO(this->get_logger(), "MPC Trajectory:");
        for (int i = 0; i < 11; ++i) {
            RCLCPP_INFO(this->get_logger(), "Point %d: x=%.3f, y=%.3f", 
                        i, trajectory[3*i], trajectory[3*i+1]);
        }

        wheel_commands_[0] = result[0];
        wheel_commands_[1] = result[1];

        RCLCPP_INFO(this->get_logger(), "MPC Inputs - Robot Position: (%.2f, %.2f, %.2f)",
            current_robot_position[0], current_robot_position[1], current_robot_position[2]);
        RCLCPP_INFO(this->get_logger(), "MPC Inputs - Goal: (%.2f, %.2f, %.2f)",
            current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);

        handle_solver_status(status);
        update_trajectory_data(trajectory);
    }

    std::array<double, 520> transform_costmap() const {
        std::array<double, 520> transformed;
        for (size_t i = 0; i < 260; ++i) {
            transformed[2*i] = costmap_data_[2*i] - current_pose_[0];
            transformed[2*i+1] = costmap_data_[2*i+1] - current_pose_[1];
        }
        return transformed;
    }

    std::array<double, 30> calculate_tracking_goal() const {
        std::array<double, 30> tracking_goal;
        for (int i = 0; i < 10; ++i) {
            tracking_goal[3*i] = goal_pose_[0] - current_pose_[0];
            tracking_goal[3*i+1] = goal_pose_[1] - current_pose_[1];
            tracking_goal[3*i+2] = goal_pose_[2];
        }
        return tracking_goal;
    }

    void handle_solver_status(int status) {
        if (status > 0) {
            RCLCPP_WARN(this->get_logger(), "Resetting MPC solver due to status: %d", status);
            myMpcSolver.reset_solver();
            myMpcSolver = my_NMPC_solver(mpc_horizon_);
        }
    }

    void update_trajectory_data(const double trajectory[33]) {
        current_trajectory_.poses.clear();
        //auto now = now(); // Current ROS time
        // gets current time once
        //auto current_time = this->now(); 
        RCLCPP_INFO(this->get_logger(), "trajectory[0-2]: %.2f, %.2f, %.2f", 
            trajectory[0], trajectory[1], trajectory[2]);
        for (int i = 0; i < 33; ++i) {
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now() + rclcpp::Duration::from_seconds(i * 1); //() + rclcpp::Duration(i * 0.1);
            pose.header.frame_id = "map";
            // current_trajectory_.header.stamp = current_time; //() + rclcpp::Duration(i * 0.1);
            // current_trajectory_.header.frame_id = "map";
            //nmpc_path_pub_->publish(current_trajectory_);

            pose.pose.position.x = trajectory[2*i] + current_pose_[0];
            pose.pose.position.y = trajectory[2*i+1] + current_pose_[1];
            current_trajectory_.poses.push_back(pose);

            pose.pose.position.x = trajectory[3*i] + current_pose_[0];
            pose.pose.position.y = trajectory[3*i+1] + current_pose_[1];
            current_trajectory_.poses.push_back(pose);
        }
        current_trajectory_.header.frame_id = "map"; // added to fix the path
        current_trajectory_.header.stamp = now(); // push the time to the header
        
        nmpc_path_pub_->publish(current_trajectory_);
    }

    void publish_trajectory() {
        current_trajectory_.header.stamp = this->now();
        current_trajectory_.header.frame_id = "map";
        nmpc_path_pub_->publish(current_trajectory_);
    }

    void publish_robot_arrow() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = goal_from_target_[0];
        msg.pose.position.y = goal_from_target_[1];
        msg.pose.position.z = goal_from_target_[2];
        msg.pose.orientation.x = goal_from_target_[3];
        msg.pose.orientation.y = goal_from_target_[4];
        msg.pose.orientation.z = goal_from_target_[5];
        msg.pose.orientation.w = goal_from_target_[6];
        robot_arrow_pub_->publish(msg);
    }

    void publish_goal_arrow() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = goal_pose_[0];
        msg.pose.position.y = goal_pose_[1];

        tf2::Quaternion q;
        q.setRPY(0, 0, goal_pose_[2]);
        msg.pose.orientation = tf2::toMsg(q.normalize());

        goal_arrow_pub_->publish(msg);
    }

    void publish_motor_commands() {
        auto command = std_msgs::msg::Float64MultiArray();
        command.data = {wheel_commands_[0], wheel_commands_[0], 
                       wheel_commands_[1], wheel_commands_[1]};
        motor_controller_->publish(command);
    }



    void log_data() {
        if (myfile_.is_open()) {
            myfile_ << current_pose_[0] << "," << current_pose_[1] << ","
                    << goal_pose_[0] << "," << goal_pose_[1] << "\n";
        }
    }

    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_arrow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_arrow_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nmpc_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_controller_;
    
    std::mutex clock_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;

    // MPC Solver
    my_NMPC_solver myMpcSolver;

    // Data storage
    std::array<double, 6> goal_pose_ = {0};
    std::array<double, 6> current_pose_ = {0};
    std::array<double, 520> costmap_data_;
    nav_msgs::msg::Path current_trajectory_;
    std::array<double, 2> wheel_commands_ = {0};

    // Configuration
    std::string goal_topic_;
    std::string costmap_topic_;
    std::string amcl_pose_topic_;
    std::string data_file_path_;
    std::string perf_file_path_;
    double control_rate_;
    int mpc_horizon_;
    int goal_status_ = 0;

    // File handling
    std::ofstream myfile_;
    std::ofstream myperffile_;

    // TF2-related
    nav_msgs::msg::MapMetaData costmap_info_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}