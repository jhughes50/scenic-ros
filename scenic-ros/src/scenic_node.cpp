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

    declare_parameter("glider_path", "");

    // Get parameters
    freq_ = this->get_parameter("publishers.rate").as_double();

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = true; //this->get_parameter("publishers.viz.use").as_bool();
    origin_.easting = 482942.0998531349; //this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_.northing = 4421353.329070162; // this->get_parameter("publishers.viz.origin_northing").as_double();
    
    bool use_odom = this->get_parameter("subscribers.use_odom").as_bool();
    
    std::string glider_path = this->get_parameter("glider_path").as_string();

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
    vo_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/scenic/odom/vo", 10);
    odom_viz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/scenic/odom/viz", 10);
    graph_str_pub_ = this->create_publisher<std_msgs::msg::String>("/scenic/graph/string", 10);
    graph_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/scenic/graph/image", 1);
    graph_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/scenic/graph/viz", 1);

    std::chrono::milliseconds vd = ScenicROS::Conversions::hzToDuration(1.0);
    vo_timer_ = this->create_wall_timer(vd, std::bind(&ScenicNode::pushCallback, this));
}

void ScenicNode::pushCallback()
{
    std::cout << "[SCENIC] [ROS] Is New Graph: " << std::boolalpha << scenic_->isNewGraph() << std::endl;
    if (scenic_->isNewGraph()) {
        graph_ = scenic_->getGraph();
        if (graph_) {
            publishGraphViz();
            cv::Mat img = scenic_->getGraphImage();
            sensor_msgs::msg::Image msg = ScenicROS::Conversions::imageToRos(img);
            graph_img_pub_->publish(msg);
        }
    }
    if (img_initialized_ && !image_stamped_.image.empty() && current_state_.isInitialized() && scenic_->isInitialized()) {
        scenic_->push(image_stamped_.stampi, image_stamped_.image);
    }
    //if (scenic_->isInitialized() && current_state_.isInitialized()) {
    //    scenic_->addImage(image_stamped_.stampd, image_stamped_.stampi, image_stamped_.image, true);
        // send the most recent image-odom pair to processors
        //LOG(INFO) << "[SCENIC] Pushing image odom pair" << std::endl;
        //if (img_counter_ > 0 && img_counter_ % 10 == 0) {
        //    scenic_->addImage(image_stamped_.stampd, image_stamped_.stampi, image_stamped_.image, true);
        //} else {
        //    scenic_->addImage(image_stamped_.stampd, image_stamped_.stampi, image_stamped_.image, false);
        //}
        //Glider::Odometry vo = scenic_->getVisualOdometry();
        //publishVisualOdometry(vo);
        // check if a new graph came out of the processors
        //if (scenic_->isNewGraph()) {
        //    cv::Mat img = scenic_->getGraphImage();
        //    sensor_msgs::msg::Image::SharedPtr msg = ScenicROS::Conversions::imageToRos(img);
        //    graph_img_pub_->publish(*msg);
        //}
    //}
}

void ScenicNode::pushVoCallback()
{ 
    // DEPRICATED
    //if (img_initialized_ && !image_stamped_.image.empty() && current_state_.isInitialized()) {
    //    scenic_->addImage(image_stamped_.stampd, image_stamped_.stampi, image_stamped_.image);
    //}
}

void ScenicNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved IMU measurement";
    Eigen::Vector3d gyro = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = ScenicROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    Glider::Odometry odom = scenic_->addIMU(timestamp, accel, gyro, orient);
    if (odom.isInitialized()) {
        publishOdometry(odom);
    }
}

void ScenicNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved GPS measurement";
    Eigen::Vector3d gps = ScenicROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    current_state_ = scenic_->addGPS(timestamp, gps);
    publishOdometry(current_state_);
}

void ScenicNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[SCENIC] Recieved Image";
    //if (!current_state_.isInitialized()) return;
    img_initialized_ = true;
    rclcpp::Time t = rclcpp::Time(msg->header.stamp);
    cv::Mat image = ScenicROS::Conversions::rosToImage(msg);
    cv::resize(image, image, cv::Size(), 0.25, 0.25);

    //int64_t timestamp = getTime(msg->header.stamp);
    //Glider::Odometry odom = glider_->interpolate(timestamp);

    //image_odom_pair_ = std::make_pair(image, odom);
    image_stamped_.image = image;
    image_stamped_.stampd = t.seconds();
    image_stamped_.stampi = getTime(msg->header.stamp);
}

void ScenicNode::textCallback(const scenic_msgs::msg::TextArray::ConstSharedPtr msg)
{
    std::vector<scenic_msgs::msg::Text> classes = msg->classes;
    std::vector<Scenic::Text> texts; 
    for (const scenic_msgs::msg::Text& txt_msg : classes) {
        Scenic::Text t(txt_msg.label,
                       static_cast<Scenic::GraphLevel>(txt_msg.level),
                       static_cast<Scenic::RegionPriority>(txt_msg.priority));
        texts.push_back(t);
    }

    scenic_->setText(texts);
}

int64_t ScenicNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
    return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void ScenicNode::publishOdometry(Glider::OdometryWithCovariance& odom) const 
{
    LOG_FIRST_N(INFO, 1) << "[ROS] Publishing Odometry from Optimizer";
    nav_msgs::msg::Odometry msg = ScenicROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
    odom_pub_->publish(msg);
    if (viz_) {
        publishOdometryViz(msg);
    }
}

void ScenicNode::publishOdometry(Glider::Odometry& odom) const 
{
    LOG_FIRST_N(INFO, 1) << "[ROS] Publishing Odometry from Inference";
    nav_msgs::msg::Odometry msg = ScenicROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
    odom_pub_->publish(msg);
    if (viz_) {
        publishOdometryViz(msg);
    }
}

void ScenicNode::publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const
{
    double x = viz_msg.pose.pose.position.x - origin_.easting;
    double y = viz_msg.pose.pose.position.y - origin_.northing;
    viz_msg.pose.pose.position.x = x;
    viz_msg.pose.pose.position.y = y;

    odom_viz_pub_->publish(viz_msg);
}

void ScenicNode::publishVisualOdometry(Glider::Odometry& odom) const
{
    nav_msgs::msg::Odometry msg = ScenicROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
    vo_pub_->publish(msg);
}

void ScenicNode::publishGraphViz() const
{
    LOG(INFO) << "[SCENIC] [ROS] Publishing New Graph";
    std::vector<visualization_msgs::msg::Marker> msgs = ScenicROS::Conversions::visualizeGraph(graph_, origin_);

    for (const visualization_msgs::msg::Marker& msg : msgs) {
        graph_viz_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ScenicROS::ScenicNode)
