#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ImuBiasCorrectionNode : public rclcpp::Node
{
public:
    ImuBiasCorrectionNode()
    : Node("imu_bias_correction_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/zed/zed_node/imu/data", 10,
            std::bind(&ImuBiasCorrectionNode::imuCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", 10,
            std::bind(&ImuBiasCorrectionNode::odomCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_corrected", 10);

        RCLCPP_INFO(this->get_logger(), "IMU Bias + Quaternion Orientation Correction Node started");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto corrected_msg = *msg;
        last_imu_msg_ = msg;

        tf2::Quaternion q_current;
        tf2::fromMsg(msg->orientation, q_current);
        q_current.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

        if (!has_stationary_reference_) return;

        if (robot_stationary_) {
            // Force angular velocity.z to 0
            corrected_msg.angular_velocity.z = 0.0;

            // Fix orientation to reference when stationary
            corrected_msg.orientation = tf2::toMsg(q_ref);
        } 
        // else if (has_motion_reference_) {
        else {
                        
            // Calculate yaw delta and apply to yaw_ref_
            double delta_yaw = yaw - yaw_previous_;
            double yaw_corrected = normalizeAngle(yaw_ref_ + delta_yaw);

            // Update reference
            yaw_ref_ = yaw_corrected;
            q_ref.setRPY(roll, pitch, yaw_corrected);
            q_ref.normalize();

            corrected_msg.orientation = tf2::toMsg(q_ref);
        }
        // Update previous yaw
        yaw_previous_ = yaw;

        imu_pub_->publish(corrected_msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double linear_x = std::abs(msg->twist.twist.linear.x);
        double angular_z = std::abs(msg->twist.twist.angular.z);

        bool stationary = (linear_x < 0.0001) && (angular_z < 0.0001);

        if (!has_stationary_reference_) {
            RCLCPP_INFO(this->get_logger(), "Saving initial reference.");
            if (last_imu_msg_) {
                tf2::fromMsg(last_imu_msg_->orientation, q_ref);
                yaw_ref_ = getYawFromQuaternion(q_ref);
                yaw_previous_ = yaw_ref_;
                has_stationary_reference_ = true;
            }
        }

        if (stationary) {
            stationary_counter_++;
            if (stationary_counter_ >= 5) {
                robot_stationary_ = true;
            }
        } else {
            stationary_counter_ = 0;
            robot_stationary_ = false;
        }
    }

    double getYawFromQuaternion(const tf2::Quaternion &q)
    {
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // === State ===
    sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;

    tf2::Quaternion q_ref = tf2::Quaternion(0.0, 0.0, 0.0, 1.0); 

    double yaw_ref_ = 0.0;
    double yaw_previous_ = 0.0;

    int stationary_counter_ = 0;
    bool robot_stationary_ = true;
    bool has_stationary_reference_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBiasCorrectionNode>());
    rclcpp::shutdown();
    return 0;
}
