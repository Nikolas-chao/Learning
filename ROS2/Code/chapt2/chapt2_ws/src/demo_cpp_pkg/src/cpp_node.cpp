#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("person_node");
    RCLCPP_INFO(node->get_logger(),"你好c++,节点");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}