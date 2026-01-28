/**
 * @file   dvl-nav-interface.cpp
 *
 * @author Braden Meyers
 * @date   Jan 2026
 */

#include "dvl_a50/dvl-nav-interface.hpp"

DVLNavInterface::DVLNavInterface()
: Node("dvl_nav_interface"), tf_buffer_(this->get_clock())
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);
    
    this->declare_parameter<std::string>("odom_frame_id", "dvl_odom");
    this->declare_parameter<std::string>("ned_odom_frame_id", "dvl_odom_ned");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<double>("orientation_variance", 0.15);
    this->declare_parameter<bool>("publish_tf", true);

    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("ned_odom_frame_id", ned_odom_frame_id_);
    this->get_parameter("child_frame_id", child_frame_id_);
    this->get_parameter("orientation_variance", orientation_var_);
    this->get_parameter("publish_tf", publish_tf_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this);

    dvl_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
        "dvl/data", qos,
        std::bind(&DVLNavInterface::dvl_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist", 10);

    dvldr_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/position", qos,
        std::bind(&DVLNavInterface::position_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl/odom", 10);

    RCLCPP_INFO(this->get_logger(), "DVL Navigation Interface started");
}

DVLNavInterface::~DVLNavInterface() {}

void DVLNavInterface::position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.header.stamp = msg->header.stamp;

    geometry_msgs::msg::TransformStamped T_dvl_to_base, T_zup_zdown;
    try {
        T_dvl_to_base = tf_buffer_.lookupTransform(child_frame_id_, msg->header.frame_id, msg->header.stamp);
        T_zup_zdown = tf_buffer_.lookupTransform(ned_odom_frame_id_, odom_frame_id_, msg->header.stamp);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    geometry_msgs::msg::Pose ned_dvl_pose_in_dvl;
    ned_dvl_pose_in_dvl.position.x = msg->position.x;
    ned_dvl_pose_in_dvl.position.y = msg->position.y;
    ned_dvl_pose_in_dvl.position.z = msg->position.z;
    ned_dvl_pose_in_dvl.orientation = quaternion_from_euler(msg->roll, msg->pitch, msg->yaw, true);

    geometry_msgs::msg::Pose ned_dvl_pose_in_base;
    tf2::doTransform(ned_dvl_pose_in_dvl, ned_dvl_pose_in_base, T_dvl_to_base);

    geometry_msgs::msg::TransformStamped ned_dvl_transform_in_base;
    ned_dvl_transform_in_base.transform.translation.x = ned_dvl_pose_in_base.position.x;
    ned_dvl_transform_in_base.transform.translation.y = ned_dvl_pose_in_base.position.y;
    ned_dvl_transform_in_base.transform.translation.z = ned_dvl_pose_in_base.position.z;
    ned_dvl_transform_in_base.transform.rotation = ned_dvl_pose_in_base.orientation;

    geometry_msgs::msg::Pose pose_zup_zdown;
    pose_zup_zdown.position.x = T_zup_zdown.transform.translation.x;
    pose_zup_zdown.position.y = T_zup_zdown.transform.translation.y;
    pose_zup_zdown.position.z = T_zup_zdown.transform.translation.z;
    pose_zup_zdown.orientation = T_zup_zdown.transform.rotation;

    geometry_msgs::msg::Pose dvl_pose_in_base;
    tf2::doTransform(pose_zup_zdown, dvl_pose_in_base, ned_dvl_transform_in_base);

    odom.pose.pose = dvl_pose_in_base;

    double var = msg->pos_std * msg->pos_std;
    odom.pose.covariance[0] = var;
    odom.pose.covariance[7] = var;
    odom.pose.covariance[14] = var;
    odom.pose.covariance[21] = orientation_var_;
    odom.pose.covariance[28] = orientation_var_;
    odom.pose.covariance[35] = orientation_var_;

    odom_pub_->publish(odom);
    if (publish_tf_)
        publish_odom_tf(odom);
}

void DVLNavInterface::publish_odom_tf(const nav_msgs::msg::Odometry &odom)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom.header.stamp;
    tf_msg.header.frame_id = odom.header.frame_id;
    tf_msg.child_frame_id = odom.child_frame_id;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
}

void DVLNavInterface::dvl_callback(const dvl_msgs::msg::DVL::SharedPtr msg)
{
    if (!msg->velocity_valid) {
        RCLCPP_WARN(this->get_logger(), "Received invalid DVL velocity");
        return;
    }
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = msg->header.stamp;
    twist_msg.header.frame_id = msg->header.frame_id;
    twist_msg.twist.twist.linear.x = msg->velocity.x;
    twist_msg.twist.twist.linear.y = msg->velocity.y;
    twist_msg.twist.twist.linear.z = msg->velocity.z;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            twist_msg.twist.covariance[i*6 + j] = msg->covariance[i*3 + j];
    twist_pub_->publish(twist_msg);
}

geometry_msgs::msg::Quaternion DVLNavInterface::quaternion_from_euler(double roll, double pitch, double yaw, bool degrees)
{
    if (degrees) {
        roll = roll * M_PI / 180.0;
        pitch = pitch * M_PI / 180.0;
        yaw = yaw * M_PI / 180.0;
    }
    double cy = cos(yaw*0.5), sy = sin(yaw*0.5);
    double cp = cos(pitch*0.5), sp = sin(pitch*0.5);
    double cr = cos(roll*0.5), sr = sin(roll*0.5);
    geometry_msgs::msg::Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DVLNavInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
