/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About ROS2 Node
*/

#include "scenic_ros/scenic_node.hpp"

using namespace ScenicROS;

ScenicNode::ScenicNode(const rclcpp::NodeOptions& options)
{
    // Declare ROS parameters
    declare_parameter("publishers.rate", 10.0);
    declare_parameter("publishers.nav_sat_fix", false);
    declare_parameter("publishers.viz.use", false);
    declare_parameter("publishers.viz.origin_easting", 0.0);
    declare_parameter("publishers.viz.origin_northing", 0.0);

    declare_parameter("subscribers.use_odom", false);

    declare_parameter("path", "");

    // Get parameters
    freq_ = this->get_parameter("publishers.rate").as_double();

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = this->get_parameter("publishers.viz.use").as_bool();
    origin_easting_ = this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_northing_ = this->get_parameter("publishers.viz.origin_northing").as_double();

    bool use_odom = this->get_parameter("subscribers.use_odom").as_bool();
    
    std::string path = this->get_parameter("path").as_string();

    // initialize glider
    glider_ = std::make_unique<Glider::Glider>(path);
}

void ScenicNode::pushCallback()
{
    //scenic_->push(image_odom_pair_.first, image_odom_pair_.second);
}

void ScenicNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharePtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENICROS] Recieved IMU measurement";
    Eigen::Vector3d gyro = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = ScenicROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addImu(timestamp, accel, gyro, orient);
}

void ScenicNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved GPS measurement";
    Eigen::Vector3d gps = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addGps(timestamp, gps);

    current_state_ = glider_->optimize(timestamp);
}

void ScenicNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!current_state_.isInitialized()) return;
    cv::Mat image = ScenicROS::Conversions::rosToImage(msg);
    
    int64_t timestamp = getTime(msg->header.stamp);
    Glider::Odometry odom = glider_->interpolate(timestamp);

    image_odom_pair_ = std::make_pair(image, odom);
}

void ScenicNode::compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
    // do I need this
}

int64_t ScenicNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
    return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void ScenicNode::publishOdometry(Glider::Odometry& odom) const
{   
    // nice to have
}

void ScenicNode::publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const
{
    // nice to have
}

void ScenicNode::publishGraph() const
{
    // need
}

