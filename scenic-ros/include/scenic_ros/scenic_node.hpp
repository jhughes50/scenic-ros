/*
* Jason Hughes
* Januray 2026
*
* ROS node header
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <glider/core/glider.hpp>
#include <glider/core/odometry.hpp>
#include <glider/core/odometry_with_covariance.hpp>

#include <scenic/core/scenic.hpp>
#include <scenic_msgs/msg/text.hpp>
#include <scenic_msgs/msg/text_array.hpp>


#include "scenic_ros/conversions.hpp"

namespace ScenicROS
{
class ScenicNode : public rclcpp::Node
{
    public:
        ScenicNode() = default;
        ScenicNode(const rclcpp::NodeOptions& options);

    private:
        std::unique_ptr<Glider::Glider> glider_;
        std::unique_ptr<Scenic::Scenic> scenic_;
    
        // timer callbacks 
        void pushCallback();
        void pushVoCallback();

        // subscriber callbacks
        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void textCallback(const scenic_msgs::msg::TextArray::ConstSharedPtr msg);

        // utility functions
        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;
        void publishOdometry(Glider::Odometry& odom) const;
        void publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const;
        void publishGraph() const;

        // subscriptions
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr img_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr cimg_sub_;
        rclcpp::Subscription<scenic_msgs::msg::TextArray>::ConstSharedPtr txt_sub_;   

        // groups
        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;
        rclcpp::CallbackGroup::SharedPtr img_group_;
        
        // publishers 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_viz_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_str_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr graph_img_pub_;

        // timers
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr vo_timer_;

        // parameters
        bool initialized_{false};
        bool img_initialized_{false};
        bool publish_nsf_;
        bool viz_;
        std::string utm_zone_;
        double origin_easting_;
        double origin_northing_;
        double freq_;

        // tracker
        Glider::OdometryWithCovariance current_state_;
        std::pair<cv::Mat, Glider::Odometry> image_odom_pair_;
        Scenic::ImageStamped image_stamped_;
};
}
