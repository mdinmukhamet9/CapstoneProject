// #include <chrono>
// #include <memory>
// #include <fstream>
// #include <iomanip>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"

// using namespace std::chrono_literals;

// class CmdVelPublisher : public rclcpp::Node
// {
// public:
//     CmdVelPublisher()
//     : Node("cmd_vel_publisher"),
//       initial_odom_received_(false),
//       cmd_sent_logged_(false),
//       movement_started_(false),
//       goal_reached_(false),
//       V0_linear_(0.0),
//       V0_angular_(0.0),
//       Vdesired_linear_(0.5),
//       Vdesired_angular_(0.0)
//     {
//         publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/j100_0001/cmd_vel", 10);
//         odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/j100_0001/platform/odom", 10,
//             std::bind(&CmdVelPublisher::odom_callback, this, std::placeholders::_1)
//         );

//         // Initialize time values to -1 (invalid)
//         cmd_sent_time_ = -1.0;
//         movement_start_time_ = -1.0;
//         goal_reach_time_ = -1.0;

//         // Open CSV file
//         log_file_.open("robot_log.csv");
//         log_file_ << "t0_command_sent,t1_movement_start,t2_goal_reached,current_time,x,y,yaw,"
//                   << "V0_linear,V0_angular,Vdesired_linear,Vdesired_angular,current_linear,current_angular"
//                   << std::endl;

//         // Setup initialization timer to wait for first odom message
//         init_timer_ = this->create_wall_timer(10ms, [this]() {
//             if (initial_odom_received_ && !cmd_sent_logged_) {
//                 publish_velocity();
//                 init_timer_->cancel();
//             }
//         });
//     }

//     ~CmdVelPublisher()
//     {
//         log_file_.close();
//     }

// private:
//     void publish_velocity()
//     {
//         auto message = geometry_msgs::msg::Twist();
//         message.linear.x = Vdesired_linear_;
//         message.angular.z = Vdesired_angular_;

//         publisher_->publish(message);
//         cmd_sent_time_ = this->get_clock()->now().seconds();
//         cmd_sent_logged_ = true;
        
//         RCLCPP_INFO(this->get_logger(), "Command sent at %.3f seconds (t0)", cmd_sent_time_);
//         RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", 
//                    message.linear.x, message.angular.z);
//     }

//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//     {
//         // Capture initial odom data before sending command
//         if (!initial_odom_received_) {
//             V0_linear_ = msg->twist.twist.linear.x;
//             V0_angular_ = msg->twist.twist.angular.z;
//             initial_odom_received_ = true;
//             return;
//         }

//         // Only process after command has been sent
//         if (!cmd_sent_logged_) return;

//         // Process odometry data
//         const double x = msg->pose.pose.position.x;
//         const double y = msg->pose.pose.position.y;
        
//         tf2::Quaternion q(
//             msg->pose.pose.orientation.x,
//             msg->pose.pose.orientation.y,
//             msg->pose.pose.orientation.z,
//             msg->pose.pose.orientation.w
//         );
//         tf2::Matrix3x3 m(q);
//         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);

//         const double linear_x = msg->twist.twist.linear.x;
//         const double angular_z = msg->twist.twist.angular.z;
//         const double now_sec = this->get_clock()->now().seconds();

//         // Detect movement start (t1) - use higher threshold to avoid noise
//         if (!movement_started_ && std::abs(linear_x - V0_linear_) > 0.05) {
//             movement_start_time_ = now_sec;
//             movement_started_ = true;
//             RCLCPP_INFO(this->get_logger(), "Movement started at t1 = %.3f sec (delay: %.3f sec)", 
//                        movement_start_time_, movement_start_time_ - cmd_sent_time_);
//         }

//         // Detect goal velocity reached (t2)
//         if (movement_started_ && !goal_reached_) {
//             const double linear_tolerance = 0.02;
//             if (std::abs(linear_x - Vdesired_linear_) < linear_tolerance) {
//                 goal_reach_time_ = now_sec;
//                 goal_reached_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Reached desired velocity at %.3f seconds (t2)", 
//                            goal_reach_time_);
                
//                 // Calculate and log results
//                 const double reaction_time = movement_start_time_ - cmd_sent_time_;
//                 const double acceleration_time = goal_reach_time_ - movement_start_time_;
//                 const double acceleration = (Vdesired_linear_ - V0_linear_) / acceleration_time;

//                 RCLCPP_INFO(this->get_logger(), "\nResults:\n"
//                              "Reaction Time: %.3f s\n"
//                              "Acceleration: %.3f m/s²", 
//                              reaction_time, acceleration);
//             }
//         }

//         // Log data
//         log_file_ << std::fixed << std::setprecision(3)
//                   << cmd_sent_time_ << ","
//                   << (movement_started_ ? movement_start_time_ : -1.0) << ","
//                   << (goal_reached_ ? goal_reach_time_ : -1.0) << ","
//                   << now_sec << ","
//                   << x << "," << y << "," << yaw << ","
//                   << V0_linear_ << "," << V0_angular_ << ","
//                   << Vdesired_linear_ << "," << Vdesired_angular_ << ","
//                   << linear_x << "," << angular_z
//                   << std::endl;

//         // Timeout check (10 seconds after command sent)
//         if (!goal_reached_ && (now_sec - cmd_sent_time_ > 10.0)) {
//             RCLCPP_ERROR(this->get_logger(), "Timeout reached without achieving goal velocity");
//             goal_reached_ = true;
//         }
//     }

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
//     rclcpp::TimerBase::SharedPtr init_timer_;
//     std::ofstream log_file_;

//     bool initial_odom_received_;
//     bool cmd_sent_logged_;
//     bool movement_started_;
//     bool goal_reached_;
    
//     double cmd_sent_time_;
//     double movement_start_time_;
//     double goal_reach_time_;
    
//     double V0_linear_;
//     double V0_angular_;
//     double Vdesired_linear_;
//     double Vdesired_angular_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CmdVelPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <chrono>
#include <memory>
#include <fstream>
#include <iomanip>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
    : Node("cmd_vel_publisher"),
      initial_odom_received_(false),
      cmd_sent_logged_(false),
      movement_started_(false),
      goal_reached_(false),
      V0_linear_(0.0),
      V0_angular_(0.0),
      Vdesired_linear_(1.2),
      Vdesired_angular_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/j100_0001/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/j100_0001/platform/odom", 10,
            std::bind(&CmdVelPublisher::odom_callback, this, std::placeholders::_1)
        );

        // Initialize time values to -1 (invalid)
        cmd_sent_time_ = -1.0;
        movement_start_time_ = -1.0;
        goal_reach_time_ = -1.0;

        // Open CSV file
        log_file_.open("robot_log.csv");
        log_file_ << "t0_command_sent,t1_movement_start,t2_goal_reached,current_time,x,y,yaw,"
                  << "V0_linear,V0_angular,Vdesired_linear,Vdesired_angular,current_linear,current_angular"
                  << std::endl;

        // Setup initialization timer to wait for first odom message
        init_timer_ = this->create_wall_timer(10ms, [this]() {
            if (initial_odom_received_ && !cmd_sent_logged_) {
                publish_velocity();
                init_timer_->cancel();
            }
        });
    }

    ~CmdVelPublisher()
    {
        log_file_.close();
    }

private:
    void publish_velocity()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = Vdesired_linear_;
        message.angular.z = Vdesired_angular_;

        publisher_->publish(message);
        cmd_sent_time_ = this->get_clock()->now().seconds();
        cmd_sent_logged_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Command sent at %.3f seconds (t0)", cmd_sent_time_);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", 
                   message.linear.x, message.angular.z);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Capture initial odom data before sending command
        if (!initial_odom_received_) {
            V0_linear_ = msg->twist.twist.linear.x;
            V0_angular_ = msg->twist.twist.angular.z;
            initial_odom_received_ = true;
            return;
        }

        // Only process after command has been sent
        if (!cmd_sent_logged_) return;

        // Process odometry data
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        const double linear_x = msg->twist.twist.linear.x;
        const double angular_z = msg->twist.twist.angular.z;
        const double now_sec = this->get_clock()->now().seconds();

        // Detect movement start (t1) - use higher threshold to avoid noise
        if (!movement_started_ && std::abs(linear_x - V0_linear_) > 0.005) {
            movement_start_time_ = now_sec;
            movement_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Movement started at t1 = %.3f sec (delay: %.3f sec)", 
                       movement_start_time_, movement_start_time_ - cmd_sent_time_);
        }

        // Detect goal velocity reached (t2)
        if (movement_started_ && !goal_reached_) {
            const double linear_tolerance = 0.00002;
            if (std::abs(linear_x - Vdesired_linear_) < linear_tolerance) {
                goal_reach_time_ = now_sec;
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached desired velocity at %.3f seconds (t2)", 
                           goal_reach_time_);
                
                // Calculate and log results
                const double reaction_time = movement_start_time_ - cmd_sent_time_;
                const double acceleration_time = goal_reach_time_ - movement_start_time_;
                const double acceleration = (Vdesired_linear_ - V0_linear_) / acceleration_time;

                RCLCPP_INFO(this->get_logger(), "\nResults:\n"
                             "Reaction Time: %.3f s\n"
                             "Acceleration: %.3f m/s²", 
                             reaction_time, acceleration);
            }
        }

        // Log data
        log_file_ << std::fixed << std::setprecision(3)
                  << cmd_sent_time_ << ","
                  << (movement_started_ ? movement_start_time_ : -1.0) << ","
                  << (goal_reached_ ? goal_reach_time_ : -1.0) << ","
                  << now_sec << ","
                  << x << "," << y << "," << yaw << ","
                  << V0_linear_ << "," << V0_angular_ << ","
                  << Vdesired_linear_ << "," << Vdesired_angular_ << ","
                  << linear_x << "," << angular_z
                  << std::endl;

        // Timeout check (10 seconds after command sent)
        if (!goal_reached_ && (now_sec - cmd_sent_time_ > 10.0)) {
            RCLCPP_ERROR(this->get_logger(), "Timeout reached without achieving goal velocity");
            goal_reached_ = true;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::ofstream log_file_;

    bool initial_odom_received_;
    bool cmd_sent_logged_;
    bool movement_started_;
    bool goal_reached_;
    
    double cmd_sent_time_;
    double movement_start_time_;
    double goal_reach_time_;
    
    double V0_linear_;
    double V0_angular_;
    double Vdesired_linear_;
    double Vdesired_angular_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}