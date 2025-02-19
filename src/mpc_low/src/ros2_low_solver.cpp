#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/msg/int32.hpp>
#include <gazebo_msgs/srv/set_model_configuration.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "acados_solver_holder.cpp" //Change later

using namespace std;

std::ofstream myfile("../../../data_low.csv", std::ios::out);
std::ofstream myperffile("../../../data_perf.csv", std::ios::out);

double gazebo_time = 0.0;

void gazebo_time_msg(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    gazebo_time = msg->clock.sec + msg->clock.nanosec * 1e-9;
}

class GoalFollower : public rclcpp::Node
{
public:
    GoalFollower() 
        : Node("goal_follower"),
          goal_pose_{0.0}, current_pose_{0.0}, costmap_data_{-100.0},
          tracking_goal_{0.0}, goal_status_(0)
    {
        goal_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/j100_0001/received_global_plan", 1,
            std::bind(&GoalFollower::goal_callback, this, std::placeholders::_1));

        costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/j100_0001/local_costmap/costmap", 1,
            std::bind(&GoalFollower::costmap_callback, this, std::placeholders::_1));

        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/j100_0001/amcl_pose", 1,
            std::bind(&GoalFollower::amcl_pose_callback, this, std::placeholders::_1));

        goal_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_arrow", 1);
        robot_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_arrow", 1);

        RCLCPP_INFO(this->get_logger(), "GoalFollower Node Initialized");
    }

    void goal_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        double trajectory_length = 0.0;
        int goal_index = 0;
        for (size_t i = 1; i < msg->poses.size(); ++i)
        {
            double x_diff = msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x;
            double y_diff = msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y;
            trajectory_length += std::sqrt(x_diff * x_diff + y_diff * y_diff);
            goal_index = i;
            if (trajectory_length >= 1.0)
            {
                break;
            }
        }

        if (goal_index > 0)
        {
            goal_status_ = 1;
            goal_pose_[0] = msg->poses[goal_index].pose.position.x;
            goal_pose_[1] = msg->poses[goal_index].pose.position.y;
            goal_pose_[2] = std::atan2(
                msg->poses[goal_index].pose.position.y - msg->poses[goal_index - 1].pose.position.y,
                msg->poses[goal_index].pose.position.x - msg->poses[goal_index - 1].pose.position.x);
        }
        else
        {
            goal_status_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory Received: Size %ld, Length %f", msg->poses.size(), trajectory_length);
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
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

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int obst_count = 0;
        double resolution = msg->info.resolution;

        for (size_t i = 0; i < msg->data.size(); i++)
        {
            if (msg->data[i] > 99)
            {
                costmap_data_[2 * obst_count] = (i % msg->info.width) * resolution + msg->info.origin.position.x;
                costmap_data_[2 * obst_count + 1] = (i / msg->info.width) * resolution + msg->info.origin.position.y;
                if (++obst_count == sizeof(costmap_data_) / sizeof(costmap_data_[0]))
                    break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Obstacles Count: %d", obst_count);
    }

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_arrow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_arrow_pub_;

    // Internal Variables
    double goal_pose_[6];
    double current_pose_[6];
    double costmap_data_[520];
    double tracking_goal_[30];
    int goal_status_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalFollower>();

    RCLCPP_INFO(node->get_logger(), "Node Started");

    // Subscriber to gazebo time
    auto gazebo_time_listener = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, [](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            // Handle clock updates if necessary
        });

    // Publishers
    auto low_level_trajectory_pub = node->create_publisher<nav_msgs::msg::Path>("mpc_solver/low_level_trajectory", 1);
    auto motor_controller_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/j100_0001/platform/cmd_vel", 1);
    auto goal_arrow_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_arrow", 1);
    auto robot_arrow_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_arrow", 1);

    my_NMPC_solver myMpcSolver(5);
    rclcpp::Rate loop_rate(40);

    while (rclcpp::ok())
    {
        double current_error = std::pow(node->goal_pose_[0] - node->current_pose_[0], 2) +
                               std::pow(node->goal_pose_[1] - node->current_pose_[1], 2) +
                               std::pow(node->goal_pose_[2] - node->current_pose_[2], 2);

        if (node->goal_status_ || current_error > 0.10)
        {
            double current_robot_position[3] = {0.0, 0.0, node->current_pose_[2]};
            double current_robot_goal[3] = {
                node->goal_pose_[0] - node->current_pose_[0],
                node->goal_pose_[1] - node->current_pose_[1],
                node->goal_pose_[2]};

            double costmap_data[520];
            std::copy(std::begin(node->costmap_data_), std::end(node->costmap_data_), std::begin(costmap_data));

            for (int i = 0; i < 260; i++)
            {
                costmap_data[2 * i] -= node->current_pose_[0];
                costmap_data[2 * i + 1] -= node->current_pose_[1];
            }

            double tracking_goal[30] = {0.0};
            for (int i = 0; i < 10; i++)
            {
                tracking_goal[i * 3] = node->goal_pose_[0] - node->current_pose_[0];
                tracking_goal[i * 3 + 1] = node->goal_pose_[1] - node->current_pose_[1];
                tracking_goal[i * 3 + 2] = node->goal_pose_[2];
            }

            double result[8] = {0.0};
            double trajectory[33] = {0.0};
            int status = myMpcSolver.solve_my_mpc(current_robot_position, costmap_data, current_robot_goal, tracking_goal, result, trajectory);

            if (status > 0)
            {
                myMpcSolver.reset_solver();
                myMpcSolver = my_NMPC_solver(5);
                RCLCPP_INFO(node->get_logger(), "Solver recreated");
            }
            if (status == 4)
            {
                std::fill(std::begin(result), std::begin(result) + 4, 0.0);
            }

            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            for (int i = 0; i < 11; i++)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = node->now();
                pose.header.frame_id = "map";
                pose.pose.position.x = trajectory[3 * i] + node->current_pose_[0];
                pose.pose.position.y = trajectory[3 * i + 1] + node->current_pose_[1];
                pose.pose.position.z = 0.0;
                path.poses.push_back(pose);
            }
            low_level_trajectory_pub->publish(path);

            std_msgs::msg::Float64MultiArray wheel_command;
            wheel_command.data = {result[0], result[0], result[1], result[1]};
            motor_controller_pub->publish(wheel_command);

            RCLCPP_INFO(node->get_logger(), "Time %f; Status %i; Error: %f", result[5], status, current_error);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    myfile.close();
    myperffile.close();
    rclcpp::shutdown();
    return 0;
}
