#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode() : Node("ekf_odom_pub")
    {
        // Initialize odometry data
        odomNew.header.frame_id = "odom";
        odomNew.pose.pose.position.z = 0;
        odomNew.pose.pose.orientation.x = 0;
        odomNew.pose.pose.orientation.y = 0;
        odomNew.twist.twist.linear.x = 0;
        odomNew.twist.twist.linear.y = 0;
        odomNew.twist.twist.linear.z = 0;
        odomNew.twist.twist.angular.x = 0;
        odomNew.twist.twist.angular.y = 0;
        odomNew.twist.twist.angular.z = 0;
        odomOld.pose.pose.position.x = initialX;
        odomOld.pose.pose.position.y = initialY;
        odomOld.pose.pose.orientation.z = initialTheta;

        // Publisher for odom data
        odom_data_pub = this->create_publisher<nav_msgs::msg::Odometry>("/j100_0001/platform/odom", 10);

        // Subscriber for initial pose
        initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/j100_0001/initialpose", 10, std::bind(&OdomPublisherNode::set_initial_2d, this, std::placeholders::_1));

        // Subscriber for joint state data
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/j100_0001/platform/joint_states", 10, std::bind(&OdomPublisherNode::update_odom, this, std::placeholders::_1));

        // Timer for updating odom at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&OdomPublisherNode::publish_odom, this));
    }

private:
    // Callback to set initial pose from the "initial_2d" topic
    void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        odomOld.pose.pose.position.x = msg->pose.position.x;
        odomOld.pose.pose.position.y = msg->pose.position.y;
        odomOld.pose.pose.orientation.z = msg->pose.orientation.z;
        initialPoseRecieved = true;
    }

    // Update odometry based on joint state data
    void update_odom(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Constants for calculation
        double B = 0.323;  // Distance between the wheels
        double alpha = 3.6; // Handpicked parameter
        double r_wheel = 0.10; // Wheel radius
        double scale = 1.0 / (alpha * B);

        // Convert incoming message to right and left velocity
        double message[2] = {0.0, 0.0};
        message[0] = (0.5 * msg->velocity[1] + 0.5 * msg->velocity[3]); // Avg of right wheels
        message[1] = (0.5 * msg->velocity[0] + 0.5 * msg->velocity[2]); // Avg of left wheels

        odomNew.header.stamp = this->now();
        double delta_time = (odomNew.header.stamp.sec - odomOld.header.stamp.sec) +
                            (odomNew.header.stamp.nanosec - odomOld.header.stamp.nanosec) / 1e9;

        // Calculate the average distance
        double cycleDistance = (0.5 * r_wheel * message[0]) + (0.5 * r_wheel * message[1]) * delta_time;

        // Calculate the number of radians the robot has turned since the last cycle
        double cycleAngle = (1.0 * scale * message[0]) + (-1.0 * scale * message[1]) * delta_time;

        // Average angle during the last cycle
        double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

        if (avgAngle > M_PI)
        {
            avgAngle -= 2 * M_PI;
        }
        else if (avgAngle < -M_PI)
        {
            avgAngle += 2 * M_PI;
        }

        // Calculate the new pose (x, y, and theta)
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
        odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

        // Prevent lockup from a single bad cycle
        if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) ||
            isnan(odomNew.pose.pose.position.z))
        {
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
            odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
        }

        // Ensure theta stays in the correct range
        if (odomNew.pose.pose.orientation.z > M_PI)
        {
            odomNew.pose.pose.orientation.z -= 2 * M_PI;
        }
        else if (odomNew.pose.pose.orientation.z < -M_PI)
        {
            odomNew.pose.pose.orientation.z += 2 * M_PI;
        }

        // Compute the velocity
        odomNew.twist.twist.linear.x = (0.5 * r_wheel * message[0]) + (0.5 * r_wheel * message[1]);
        odomNew.twist.twist.angular.z = (1.0 * scale * message[0]) + (-1.0 * scale * message[1]);

        // Save the pose data for the next cycle
        odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
        odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
        odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
        odomOld.header.stamp = odomNew.header.stamp;

        // Publish the odometry message
        odom_data_pub->publish(odomNew);
    }

    // Timer callback to publish odom periodically
    void publish_odom()
    {
        // Publish the odometry message
        odom_data_pub->publish(odomNew);
    }

    // Odometry data
    nav_msgs::msg::Odometry odomNew;
    nav_msgs::msg::Odometry odomOld;

    // Constants
    const double initialX = 0.0;
    const double initialY = 0.0;
    const double initialTheta = 0.00000000001;

    bool initialPoseRecieved = false;

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
