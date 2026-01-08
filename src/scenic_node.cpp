/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About ROS2 Node
*/

#include "scenic_ros/scenic_node.hpp"

using namespace ScenicROS;

ScenicNode::ScenicNode(const rclcpp::NodeOptions& options) : Node("scenic_node")
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
    
    std::string glider_path = this->get_parameter("glider_path").as_string();

    // initialize glider
    glider_ = std::make_unique<Glider::Glider>(glider_path);
    // initialize scenic
    scenic_ = std::make_unique<Scenic::Scenic>(10, "/home/jason/clipper/models", "/home/jason/clipper/config");
    // TODO initialize scenic as a unique ptr
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    img_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // initialize pubs and subscribers
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 20, 
                                                                std::bind(&ScenicNode::imuCallback, this, std::placeholders::_1),
                                                                imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 1, 
                                                                      std::bind(&ScenicNode::gpsCallback, this, std::placeholders::_1),
                                                                      gps_sub_options);
    auto img_sub_options = rclcpp::SubscriptionOptions();
    img_sub_options.callback_group = img_group_;
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 1, 
                                                                  std::bind(&ScenicNode::imageCallback, this, std::placeholders::_1),
                                                                  img_sub_options);

    txt_sub_ = this->create_subscription<scenic_msgs::msg::TextArray>("/text", 1, std::bind(&ScenicNode::textCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/scenic/odom", 10);
    graph_str_pub_ = this->create_publisher<std_msgs::msg::String>("scenic/graph", 10);
}

void ScenicNode::pushCallback()
{
    //scenic_->push(image_odom_pair_.first, image_odom_pair_.second);
}

void ScenicNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved IMU measurement";
    Eigen::Vector3d gyro = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = ScenicROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addImu(timestamp, accel, gyro, orient);
}

void ScenicNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved GPS measurement";
    Eigen::Vector3d gps = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addGps(timestamp, gps);

    current_state_ = glider_->optimize(timestamp);
}

void ScenicNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved Image";
    if (!current_state_.isInitialized()) return;
    cv::Mat image = ScenicROS::Conversions::rosToImage(msg);
    
    int64_t timestamp = getTime(msg->header.stamp);
    Glider::Odometry odom = glider_->interpolate(timestamp);

    image_odom_pair_ = std::make_pair(image, odom);
}

void ScenicNode::textCallback(const scenic_msgs::msg::TextArray::ConstSharedPtr msg)
{
    std::vector<scenic_msgs::msg::Text> classes = msg->classes;
    std::vector<Scenic::Text> texts; 
    for (const Text& class : classes) {
        Scenic::Text t(class.label,
                       static_cast<Scenic::GraphLevel>(class.level),
                       static_cast<Scenic::RegionPriorty>(class.priority));
        texts.push_back(t);
    }

    scenic->setText(texts);
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

RCLCPP_COMPONENTS_REGISTER_NODE(ScenicROS::ScenicNode)
