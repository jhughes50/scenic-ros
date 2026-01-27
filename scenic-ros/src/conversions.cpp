/*
* Jason Hughes
* July 2025 
*
* convert between ros and eigen
*/

#include "scenic_ros/conversions.hpp"

using namespace ScenicROS;

template<typename Output, typename Input>
Output Conversions::rosToEigen(const Input& msg)
{
    if constexpr (std::is_same_v<Input, geometry_msgs::msg::Vector3>)
    {
        return RosToEigen::vector3Convert(msg);
    }
    else if constexpr (std::is_same_v<Input, geometry_msgs::msg::Quaternion>)
    {
        return RosToEigen::orientConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, sensor_msgs::msg::NavSatFix>)
    {
        return RosToEigen::gpsConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, geometry_msgs::msg::PoseStamped>)
    {
        return RosToEigen::poseConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, nav_msgs::msg::Odometry>)
    {
        return RosToEigen::odomConvert(msg);
    }
}

template<typename Output, typename Input>
Output Conversions::eigenToRos(const Input& vec)
{
    if constexpr (std::is_same_v<Input, Eigen::Vector3d> && std::is_same_v<Output, geometry_msgs::msg::Vector3>)
    {
        return EigenToRos::vector3Convert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Vector3d> && std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        return EigenToRos::gpsConvert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Vector4d>)
    {
        return EigenToRos::orientConvert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Isometry3d>)
    {
        return EigenToRos::poseConvert(vec);
    }
    else
    {
        static_assert(std::is_same_v<Input, Eigen::Vector3d> ||
                      std::is_same_v<Input, Eigen::Vector4d> ||
                      std::is_same_v<Input, Eigen::Isometry3d>, "Unsupported Eigen type");
        static_assert(std::is_same_v<Output, geometry_msgs::msg::Vector3> ||
                      std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      std::is_same_v<Output, geometry_msgs::msg::Quaternion> ||
                      std::is_same_v<Output, geometry_msgs::msg::PoseStamped>, "Unsupported Ros Msg type");
    }
}

Eigen::Isometry3d Conversions::RosToEigen::poseConvert(const geometry_msgs::msg::PoseStamped& msg)
{
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = quat.toRotationMatrix();
    iso.translation() = trans;

    return iso;
}

Eigen::Vector3d Conversions::RosToEigen::vector3Convert(const geometry_msgs::msg::Vector3& msg)
{
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}   

Eigen::Vector4d Conversions::RosToEigen::orientConvert(const geometry_msgs::msg::Quaternion& msg)
{
    return Eigen::Vector4d(msg.w, msg.x, msg.y, msg.z);
}

Eigen::Vector3d Conversions::RosToEigen::gpsConvert(const sensor_msgs::msg::NavSatFix& msg)
{
    return Eigen::Vector3d(msg.latitude, msg.longitude, msg.altitude);
}

Eigen::Isometry3d Conversions::RosToEigen::odomConvert(const nav_msgs::msg::Odometry& msg)
{ 
    Eigen::Quaterniond quat(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = quat.toRotationMatrix();
    iso.translation() = trans;

    return iso;
}

geometry_msgs::msg::Vector3 Conversions::EigenToRos::vector3Convert(const Eigen::Vector3d& vec)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = vec(0);
    msg.y = vec(1);
    msg.z = vec(2);

    return msg;
}

sensor_msgs::msg::NavSatFix Conversions::EigenToRos::gpsConvert(const Eigen::Vector3d& vec)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = vec(0);
    msg.longitude = vec(1);
    msg.altitude = vec(2);

    return msg;
}

geometry_msgs::msg::Quaternion Conversions::EigenToRos::orientConvert(const Eigen::Vector4d& vec)
{
    geometry_msgs::msg::Quaternion msg;

    msg.w = vec(0);
    msg.x = vec(1);
    msg.y = vec(2);
    msg.z = vec(3);

    return msg;
}

geometry_msgs::msg::PoseStamped Conversions::EigenToRos::poseConvert(const Eigen::Isometry3d& vec)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = vec.translation().x();
    msg.pose.position.y = vec.translation().y();
    msg.pose.position.z = vec.translation().z();

    Eigen::Quaterniond quat(vec.rotation());
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();

    return msg;
}

std_msgs::msg::Header Conversions::getHeader(int64_t timestamp, std::string frame)
{
    std_msgs::msg::Header msg;
    msg.frame_id = frame;
    msg.stamp.sec = static_cast<int32_t>(timestamp / 1000000000LL);
    msg.stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000LL);

    return msg;
}

template<typename Output>
Output Conversions::odomToRos(Glider::Odometry& odom, const char* zone)
{
    if constexpr (std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        sensor_msgs::msg::NavSatFix msg;
        if (zone == nullptr)
        {
            throw std::invalid_argument("specify a zone for UTM to GPS converstion");
        }
        else
        {
            std::pair<double, double> latlon = odom.getLatLon(zone);

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = odom.getAltitude();
            msg.position_covariance_type = 3;
            msg.header = getHeader(odom.getTimestamp(), "enu");
        }
        return msg;
    }
    else if constexpr (std::is_same_v<Output, nav_msgs::msg::Odometry>)
    { 
        nav_msgs::msg::Odometry msg;

        Eigen::Vector3d p = odom.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = odom.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::Vector3d v = odom.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);
        msg.header = getHeader(odom.getTimestamp(), "enu");

        return msg;
    }
    else
    {
        static_assert(std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      std::is_same_v<Output, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template<typename Output>
Output Conversions::odomToRos(Glider::OdometryWithCovariance& odom_wc, const char* zone)
{
    if constexpr (std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        sensor_msgs::msg::NavSatFix msg;
        if (zone == nullptr)
        {
            throw std::invalid_argument("specify a zone for UTM to GPS conversion");
        }
        else
        {
            std::pair<double, double> latlon = odom_wc.getLatLon(zone);

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = odom_wc.getAltitude();
            msg.position_covariance_type = 3;
            Eigen::Matrix3d cov = odom_wc.getPositionCovariance();
            for (int i = 0; i < cov.rows(); ++i) 
            {
                for (int j = 0; j < cov.cols(); ++j) 
                {
                    msg.position_covariance[i * 3 + j] = cov(i, j);
                }
            }
            msg.header = getHeader(odom_wc.getTimestamp(), "enu");
        }
        return msg;
    }
    else if constexpr (std::is_same_v<Output, nav_msgs::msg::Odometry>)
    {
        nav_msgs::msg::Odometry msg;

        Eigen::Vector3d p = odom_wc.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = odom_wc.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::MatrixXd cov = odom_wc.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }

        Eigen::Vector3d v = odom_wc.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);
        
        cov = odom_wc.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        msg.header = getHeader(odom_wc.getTimestamp(), "enu");
        return msg;
    }
    else
    {
        static_assert(!std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      !std::is_same_v<Output, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

std::chrono::milliseconds Conversions::hzToDuration(double freq)
{
    if (freq <= 0)
    {
        std::cerr << "Invalid frequency: "<< freq << " Hz. Using 1 Hz instead" << std::endl;
        freq = 1.0;
    }

    double period_seconds = 1.0 / freq;
    
    std::chrono::milliseconds period_ms = std::chrono::milliseconds(static_cast<int64_t>(period_seconds * 1e3));
    return period_ms;
}

template<typename T>
void Conversions::addCovariance(const Glider::OdometryWithCovariance& odom_wc, T& msg)
{
    if constexpr (std::is_same_v<T, sensor_msgs::msg::NavSatFix>)
    {
        Eigen::Matrix3d cov = odom_wc.getPositionCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.position_covariance[i * 3 + j] = cov(i, j);
            }
        }
    }
    else if constexpr (std::is_same_v<T, nav_msgs::msg::Odometry>)
    { 
        Eigen::MatrixXd cov = odom_wc.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        cov = odom_wc.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::msg::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

cv::Mat Conversions::rosToImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv::Mat bayer(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2RGB);
    return bgr.clone();
}

sensor_msgs::msg::Image Conversions::imageToRos(const cv::Mat& img)
{
    sensor_msgs::msg::Image msg;

    msg.header.frame_id = "image";

    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = "bgr8";

    msg.step = img.step[0];
    msg.is_bigendian = false;

    size_t data_size = img.step[0] * img.rows;
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), img.data, data_size);

    return msg;
}

std::vector<visualization_msgs::msg::Marker> Conversions::visualizeGraph(const std::shared_ptr<Scenic::Graph>& graph, const Scenic::UTMPoint& origin)
{
    std::vector<visualization_msgs::msg::Marker> msgs;

    // Region Nodes
    visualization_msgs::msg::Marker region_points;
    region_points.header = getHeader(0, "enu");
    region_points.ns = "region_points";
    region_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    region_points.action = visualization_msgs::msg::Marker::ADD;
    region_points.scale.x = 0.4;
    region_points.scale.y = 0.4;
    region_points.scale.z = 0.4;

    region_points.id = 0;
    region_points.color.r = 0.0;
    region_points.color.g = 0.25;
    region_points.color.b = 0.75;
    region_points.color.a = 1.0;
    for (const std::shared_ptr<Scenic::Node>& node : graph->getRegionNodes()) {
        geometry_msgs::msg::Point p;
        Scenic::UTMPoint node_pos = node->getUtmCoordinate();
        p.x = node_pos.easting - origin.easting;
        p.y = node_pos.northing - origin.northing;
        p.z = 5.0;
        region_points.points.push_back(p);
    }
    msgs.push_back(region_points);

    // Object Nodes
    visualization_msgs::msg::Marker object_points; 
    object_points.header = getHeader(0, "enu");
    object_points.ns = "object_points";
    object_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    object_points.action = visualization_msgs::msg::Marker::ADD;
    object_points.scale.x = 0.4;
    object_points.scale.y = 0.4;
    object_points.scale.z = 0.4;

    object_points.id = 1;
    object_points.color.r = 0.0;
    object_points.color.g = 0.75;
    object_points.color.b = 0.25;
    object_points.color.a = 1.0;
    for (const std::shared_ptr<Scenic::Node>& node : graph->getObjectNodes()) {
        geometry_msgs::msg::Point p;
        Scenic::UTMPoint node_pos = node->getUtmCoordinate();
        p.x = node_pos.easting - origin.easting;
        p.y = node_pos.northing - origin.northing;
        p.z = 0.0;
        object_points.points.push_back(p);
    }
    msgs.push_back(object_points);

    // getEdges
    visualization_msgs::msg::Marker edges;
    edges.header = getHeader(0, "enu");
    edges.ns = "edges";
    edges.id = 0;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = 0.05;  // Line width
    edges.color.r = 1.0;
    edges.color.g = 1.0;
    edges.color.b = 0.0;
    edges.color.a = 1.0;
    for (const auto& [nodes, edge] : graph->getEdges()) {
        std::pair<std::shared_ptr<Scenic::Node>, std::shared_ptr<Scenic::Node>> node_pair = edge->getNodePair();
        
        geometry_msgs::msg::Point p1;
        Scenic::UTMPoint pos1 = node_pair.first->getUtmCoordinate();
        p1.x = pos1.easting - origin.easting;
        p1.y = pos1.northing - origin.northing;
        if (node_pair.first->getNodeLevel() == Scenic::GraphLevel::REGION) {
            p1.z = 5.0;
        } else {
            p1.z = 0.0;
        }

        geometry_msgs::msg::Point p2;
        Scenic::UTMPoint pos2 = node_pair.second->getUtmCoordinate();
        p2.x = pos2.easting - origin.easting;
        p2.y = pos2.northing - origin.northing;
        if (node_pair.second->getNodeLevel() == Scenic::GraphLevel::REGION) {
            p2.z = 5.0;
        } else {
            p2.z = 0.0;
        }

        edges.points.push_back(p1);
        edges.points.push_back(p2);
    }
    msgs.push_back(edges);

    return msgs;
}

std_msgs::msg::String Conversions::stringToRos(const std::string& str)
{
    std_msgs::msg::String msg;
    msg.data = str;

    return msg;
}

template Eigen::Vector3d Conversions::rosToEigen<Eigen::Vector3d>(const geometry_msgs::msg::Vector3& msg);
template Eigen::Vector3d Conversions::rosToEigen<Eigen::Vector3d>(const sensor_msgs::msg::NavSatFix& msg);
template Eigen::Vector4d Conversions::rosToEigen<Eigen::Vector4d>(const geometry_msgs::msg::Quaternion& msg);
template Eigen::Isometry3d Conversions::rosToEigen<Eigen::Isometry3d>(const geometry_msgs::msg::PoseStamped& msg);
template Eigen::Isometry3d Conversions::rosToEigen<Eigen::Isometry3d>(const nav_msgs::msg::Odometry& msg);

template geometry_msgs::msg::Vector3 Conversions::eigenToRos<geometry_msgs::msg::Vector3>(const Eigen::Vector3d& vec);
template geometry_msgs::msg::Quaternion Conversions::eigenToRos<geometry_msgs::msg::Quaternion>(const Eigen::Vector4d& vec);
template sensor_msgs::msg::NavSatFix Conversions::eigenToRos<sensor_msgs::msg::NavSatFix>(const Eigen::Vector3d& vec);
template geometry_msgs::msg::PoseStamped Conversions::eigenToRos<geometry_msgs::msg::PoseStamped>(const Eigen::Isometry3d& vec);

template nav_msgs::msg::Odometry Conversions::odomToRos<nav_msgs::msg::Odometry>(Glider::Odometry& odom, const char* zone);
template sensor_msgs::msg::NavSatFix Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(Glider::Odometry& odom, const char* zone);

template nav_msgs::msg::Odometry Conversions::odomToRos<nav_msgs::msg::Odometry>(Glider::OdometryWithCovariance& odom_wc, const char* zone);
template sensor_msgs::msg::NavSatFix Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(Glider::OdometryWithCovariance& odom_wc, const char* zone);

template void Conversions::addCovariance<nav_msgs::msg::Odometry>(const Glider::OdometryWithCovariance& odom_wc, nav_msgs::msg::Odometry& msg);
template void Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(const Glider::OdometryWithCovariance& odom_wc, sensor_msgs::msg::NavSatFix& msg);
