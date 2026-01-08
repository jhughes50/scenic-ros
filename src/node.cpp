/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About start the scenic ros node
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "scenic_ros/scenic_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    try {
        auto node = std::make_shared<ScenicROS::ScenicNode>(options);
        rclcpp::executors::MultiThreadedExecutor executor;

        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::cerr << "[SCENIC] Caught Error " << e.what() << std::endl;
    }
    return 0;
}
 
