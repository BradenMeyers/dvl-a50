#ifndef DVL_NAV_INTERFACE_HPP_
#define DVL_NAV_INTERFACE_HPP_

#include <memory>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class DVLNavInterface : public rclcpp::Node
{
public:
    DVLNavInterface();
    ~DVLNavInterface();

private:
    void position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg);
    void dvl_callback(const dvl_msgs::msg::DVL::SharedPtr msg);
    void publish_odom_tf(const nav_msgs::msg::Odometry &odom);
    geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw, bool degrees);

    std::string odom_frame_id_;
    std::string child_frame_id_;
    double orientation_var_;
    bool publish_tf_;

    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvldr_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // DVL_NAV_INTERFACE_HPP_